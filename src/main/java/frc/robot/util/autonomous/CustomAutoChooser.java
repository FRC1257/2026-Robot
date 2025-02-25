package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CustomAutoChooser {
  public static enum StartPositions {
    s1,
    s2,
    s3
  }

  public static enum ReefPositions {
    r1,
    r2,
    r3,
    r4,
    r5,
    r6,
    r7,
    r8,
    r9,
    r10,
    r11,
    r12,
    NONE
  }

  public static enum CoralStationPositions {
    c1,
    c2
  }

  public static enum ReefLevels {
    l1,
    l2,
    l3
  }

  private LoggedDashboardChooser<StartPositions> startChooser;
  private LoggedDashboardChooser<ReefPositions>[] positionChoosers = new LoggedDashboardChooser[5];
  private LoggedDashboardChooser<ReefLevels>[] levelChoosers = new LoggedDashboardChooser[5];

  private Drive drive;
  private RobotContainer robotContainer;

  public CustomAutoChooser(RobotContainer robotContainer, Drive drive) {
    this.drive = drive;
    this.robotContainer = robotContainer;

    startChooser = new LoggedDashboardChooser<>("Starting Position ");
    startChooser.addDefaultOption("Starting Position 1", StartPositions.s1);
    startChooser.addOption("Starting Position 2", StartPositions.s2);
    startChooser.addOption("Starting Position 3", StartPositions.s3);

    for (int i = 0; i < positionChoosers.length; i++) {
      positionChoosers[i] = new LoggedDashboardChooser<>("Position " + (i + 1));
      positionChoosers[i].addDefaultOption("", ReefPositions.NONE);

      // add all enum options to the chooser
      for (ReefPositions position : ReefPositions.values()) {
        if (position == ReefPositions.NONE) continue;
        positionChoosers[i].addOption(position.toString(), position);
      }
    }

    for (int i = 0; i < levelChoosers.length; i++) {
      levelChoosers[i] = new LoggedDashboardChooser<>("Position " + (i + 1) + " Level");

      // add all enum options to the chooser
      for (ReefLevels level : ReefLevels.values()) {
        levelChoosers[i].addOption(level.toString(), level);
      }
    }
  }

  public Command getAutoCommand() {
    StartPositions startPos = startChooser.get();
    ArrayList<ReefPositions> reefPoses = new ArrayList<ReefPositions>();
    ArrayList<ReefLevels> reefLevels = new ArrayList<ReefLevels>();
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    Pose2d startPose2d;
    switch (startPos) {
      case s1:
        startPose2d = FieldConstants.StartingPositions.startPos1;
        break;
      case s2:
        startPose2d = FieldConstants.StartingPositions.startPos2;
        break;
      case s3:
        startPose2d = FieldConstants.StartingPositions.startPos3;
        break;
      default:
        startPose2d = new Pose2d();
        break;
    }

    commandGroup.addCommands(
        new InstantCommand(
            () -> {
              drive.setPose(startPose2d);
            },
            drive));

    // make a ReefPos list that skips over all the NONE positions
    for (int i = 0; i < positionChoosers.length; i++) {
      ReefPositions reefPos = positionChoosers[i].get();
      if (reefPos == ReefPositions.NONE) continue;

      // add the positions that are not NONE to the arraylists
      reefPoses.add(reefPos);
      reefLevels.add(levelChoosers[i].get());
    }

    if (reefPoses.size() == 0) return commandGroup;

    // startpos to reef point
    commandGroup.addCommands(
        drive.followPathFileCommand(startPos.toString() + "-" + reefPoses.get(0).toString()));

    commandGroup.addCommands();

    for (int i = 1; i < reefPoses.size(); i++) {
      ReefPositions currentReefPosition = reefPoses.get(i - 1);
      ReefLevels currentReefLevel = reefLevels.get(i - 1);
      ReefPositions nextReefPosition = reefPoses.get(i);
      ReefLevels nextReefLevel = reefLevels.get(i);

      CoralStationPositions coralStationPos;
      switch (currentReefPosition) {
          // top reef spots
        case r2:
        case r3:
        case r4:
        case r5:
        case r6:
        case r7:
          coralStationPos = CoralStationPositions.c1;
          break;
        case r8:
        case r9:
        case r10:
        case r11:
        case r12:
        case r1:
          coralStationPos = CoralStationPositions.c2;
          break;
        default:
          // unreachable (all r spots are acounted for)
          System.out.println("Bug: Autochooser tried to navigate to a nonexistent reef spot");
          coralStationPos = CoralStationPositions.c1;
          break;
      }

      Command fullLevelAutoCommand = new InstantCommand();
      switch (currentReefLevel) {
        case l1:
          fullLevelAutoCommand = robotContainer.fullL1Auto();
          break;
        case l2:
          fullLevelAutoCommand = robotContainer.fullL2Auto();
          break;
        case l3:
          fullLevelAutoCommand = robotContainer.fullL3Auto();
          break;
      }

      commandGroup.addCommands(
          fullLevelAutoCommand,
          // go to coral station
          drive.followPathFileCommand(
              currentReefPosition.toString() + "-" + coralStationPos.toString()),
          robotContainer.goToStationAuto(),
          robotContainer.coralIntake(),
          // go bacak to reef
          drive.followPathFileCommand(
              coralStationPos.toString() + "-" + nextReefPosition.toString()));
    }
    return commandGroup;
  }
}
