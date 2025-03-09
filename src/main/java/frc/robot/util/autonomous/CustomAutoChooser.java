package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.drive.AllianceFlipUtil;
import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CustomAutoChooser {
  public static enum StartPositions {
    s1,
    s2,
    s3,
    s4,
    s5,
    s6
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

  /*dropdown for each one in elastic, allows us to choose between start, position,
  and level*/
  private LoggedDashboardChooser<StartPositions> startChooser;
  private LoggedDashboardChooser<ReefPositions>[] positionChoosers = new LoggedDashboardChooser[5];
  private LoggedDashboardChooser<ReefLevels>[] levelChoosers = new LoggedDashboardChooser[5];

  private Drive drive;
  private RobotContainer robotContainer;

  public CustomAutoChooser(RobotContainer robotContainer, Drive drive) {
    this.drive = drive;
    this.robotContainer = robotContainer;

    // initializes the choosers
    startChooser = new LoggedDashboardChooser<>("Starting Position ");
    startChooser.addDefaultOption("s1", StartPositions.s1);
    startChooser.addOption("s2", StartPositions.s2);
    startChooser.addOption("s3", StartPositions.s3);
    startChooser.addOption("s4", StartPositions.s4);
    startChooser.addOption("s5", StartPositions.s5);
    startChooser.addOption("s6", StartPositions.s6);

    // add all options for position
    for (int i = 0; i < positionChoosers.length; i++) {
      positionChoosers[i] = new LoggedDashboardChooser<>("Reef Position " + (i + 1));
      positionChoosers[i].addDefaultOption("", ReefPositions.NONE);

      // add all enum options to the chooser
      for (ReefPositions position : ReefPositions.values()) {
        if (position == ReefPositions.NONE) continue;
        positionChoosers[i].addOption(position.toString(), position);
      }
    }

    // add all options for levels
    for (int i = 0; i < levelChoosers.length; i++) {
      levelChoosers[i] = new LoggedDashboardChooser<>("Reef Level " + (i + 1));
      // add all enum options to the chooser
      for (ReefLevels level : ReefLevels.values()) {
        levelChoosers[i].addOption(level.toString(), level);
      }
    }
  }

  // Returns the elevator and coral pivot command to go to the desired level
  public Command getElevatorAndPivotCommand(ReefLevels level) {
    switch (level) {
      case l1:
        return robotContainer.goToL1Auto().alongWith(robotContainer.coralIntake());
      case l2:
        return robotContainer.goToL2Auto();
      case l3:
        return robotContainer.goToL3Auto();
    }
    return new InstantCommand(); //
  }

  // returns the entire auto
  public Command getAutoCommand() {
    StartPositions startPos = startChooser.get();
    ArrayList<ReefPositions> reefPoses = new ArrayList<ReefPositions>();
    ArrayList<ReefLevels> reefLevels = new ArrayList<ReefLevels>();
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // creates start position to reset our estimates
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
      case s4:
        startPose2d = FieldConstants.StartingPositions.startPos4;
        break;
      case s5:
        startPose2d = FieldConstants.StartingPositions.startPos5;
        break;
      case s6:
        startPose2d = FieldConstants.StartingPositions.startPos6;
        break;
      default:
        startPose2d = new Pose2d();
        break;
    }

    // Reset drive position so pose estimator doesn't tweak out
    commandGroup.addCommands(
        new InstantCommand(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(startPose2d));
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

    // Drive from start position to first reef position and drop preloaded coral
    Command driveStartToReef =
        drive.followPathFileCommand(startPos.toString() + "-" + reefPoses.get(0).toString());

    commandGroup.addCommands(
        driveStartToReef.alongWith(getElevatorAndPivotCommand(reefLevels.get(0))),
        (robotContainer.coralOuttake()));

    // returns poses for reef positions
    for (int i = 1; i < reefPoses.size(); i++) {
      ReefPositions currentReefPosition = reefPoses.get(i - 1);
      ReefPositions nextReefPosition = reefPoses.get(i);
      ReefLevels reefLevel = reefLevels.get(i);

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

      // drives to the stations
      Command driveStationCommand =
          drive.followPathFileCommand(
              currentReefPosition.toString() + "-" + coralStationPos.toString());

      // drives to the reef position chosen
      Command driveReefCommand =
          drive.followPathFileCommand(
              coralStationPos.toString() + "-" + nextReefPosition.toString());

      // combines all the commands needed
      commandGroup.addCommands(
          driveStationCommand.alongWith(robotContainer.goToStationAuto()),
          robotContainer.coralIntake(),
          driveReefCommand.alongWith(getElevatorAndPivotCommand(reefLevel)),
          robotContainer.coralOuttake());
    }
    return commandGroup;
  }
}
