package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperPivot.HopperPivotConstants;
import frc.robot.subsystems.HopperPivot.HopperPivot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.drive.AllianceFlipUtil;
import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CustomAutoChooser {

    // taking into account 3 positions on each side 
    // in front of tower, outpost, or depot
public static enum StartPositions {
    s1,
    s2,
    s3,
    s4,
    s5,
    s6
  }

public static enum TowerLevel {
    l1,
    l2,
    l3
  }


public static enum Actions {
    intake,
    shoot, 
    climb,
    pivot,
    NONE
}
  //either facing up or down
public static enum HopperPosition {
    up,
    down
}

public static enum ScoringPositions {
    p1, p2, p3
}

public static enum intakePostions {
    outpost,
    depot
}




private LoggedDashboardChooser<StartPositions> startChooser;
  private LoggedDashboardChooser<HopperPosition>[] hopperChoosers = new LoggedDashboardChooser[5];
  private LoggedDashboardChooser<intakePostions>[] intakeChoosers = new LoggedDashboardChooser[5];
  private LoggedDashboardChooser<TowerLevel>[] levelChoosers = new LoggedDashboardChooser[5];
  private LoggedDashboardChooser<Actions>[] actionChooser = new LoggedDashboardChooser[5];
  private LoggedDashboardChooser<ScoringPositions>[] scoringPChooser = new LoggedDashboardChooser[5];
  private Drive drive;
  private RobotContainer robotContainer;
  private HopperPivot hopperPivot;


  private boolean[] fuelShot = new boolean[6];

  public CustomAutoChooser(RobotContainer robotContainer, Drive drive, HopperPivot hopperPivot) {
    this.drive = drive;
    this.robotContainer = robotContainer;
    this.hopperPivot = hopperPivot;

    // initializes the choosers
    startChooser = new LoggedDashboardChooser<>("Starting Position ");
    startChooser.addDefaultOption("s1", StartPositions.s1);
    startChooser.addOption("s2", StartPositions.s2);
    startChooser.addOption("s3", StartPositions.s3);
    startChooser.addOption("s4", StartPositions.s4);
    startChooser.addOption("s5", StartPositions.s5);
    startChooser.addOption("s6", StartPositions.s6);




    for (int i = 0; i < 5; i++) {
        actionChooser[i] = new LoggedDashboardChooser<>("Action " + (i + 1));
        actionChooser[i].addDefaultOption("NONE", Actions.NONE);
        for (Actions pos : Actions.values()) {
            if (pos != Actions.NONE) {
                actionChooser[i].addOption(pos.name(), pos);
            }
        }

        intakeChoosers[i] = new LoggedDashboardChooser<>("Intake " + (i + 1));
        intakeChoosers[i].addDefaultOption("outpost", intakePostions.outpost);
        for (intakePostions pos : intakePostions.values()) {
            if (pos != intakePostions.outpost) {
                intakeChoosers[i].addOption(pos.name(), pos);
            }
        }

        levelChoosers[i] = new LoggedDashboardChooser<>("Tower Level " + (i + 1));
        levelChoosers[i].addDefaultOption("l1", TowerLevel.l1);
        for (TowerLevel level : TowerLevel.values()) {
            if (level != TowerLevel.l1) {
                levelChoosers[i].addOption(level.name(), level);
            }
        }
        hopperChoosers[i] = new LoggedDashboardChooser<>("Hopper Position " + (i + 1));
        hopperChoosers[i].addDefaultOption("down", HopperPosition.down);
        for (HopperPosition hopperPos : HopperPosition.values()) {
            if (hopperPos != HopperPosition.down) {
                hopperChoosers[i].addOption(hopperPos.name(), hopperPos);
            }
        }
        scoringPChooser[i] = new LoggedDashboardChooser<>("Scoring Position " + (i + 1));
        scoringPChooser[i].addDefaultOption("p1", ScoringPositions.p1);
        for (ScoringPositions pos : ScoringPositions.values()) {
            if (pos != ScoringPositions.p1) {
                scoringPChooser[i].addOption(pos.name(), pos);
            }
        }
    }
  }
    //extra commands prolly needed for coding (ik we're going to l1 but ig)
    public Command getClimbCommand(TowerLevel level) {
    switch (level) {
      case l1:
        //Commands THAT GOES TO L1
      case l2:
        //commands THAT GOES TO L2
      case l3:
        //commands THAT GOES TO L3
    }
    return Commands.none();
    }

    public Command getHopperPivot(HopperPosition state) {
    if (state == HopperPosition.down) {
      //return command to get hopper to up position
    } else {
      //send hopper to down position
    }
    return Commands.none(); //to be deleted later i js want the error to go away for my sanity
    } 

    public Command getIntakeCommand(intakePostions pos) {
        switch (pos) {
            case depot:
                //commands to intake from depot
            case outpost:
                //commands to intake from outpost
        }
        return Commands.none();
    }

    public Command getShootCommand(ScoringPositions pos){
    switch (pos) {
      case p1:
        //commands to shoot from p1
      case p2:
        //commands to shoot from p2
      case p3:
        //commands to shoot from p3
    }
    return Commands.none();
    }
    public Command getAutoCommand() {
    StartPositions startPos = startChooser.get();

    SequentialCommandGroup commandGroup = new SequentialCommandGroup();



    // creates start position to reset our estimates
    Pose2d startPose2d; //need to change these in our field constants to match the actual starting positions
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

    commandGroup.addCommands(
        new InstantCommand(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(startPose2d));
            },
            drive) //,
        //command to put the hopper down so we can intake fuel
    );

    for (int i = 0; i < 5; i++) {
        Actions action = actionChooser[i].get();
        if (action == Actions.NONE) continue;

        TowerLevel level = levelChoosers[i].get();
        HopperPosition hopper = hopperChoosers[i].get();
        ScoringPositions scorePos = scoringPChooser[i].get();

        switch (action) {
            case shoot:
                commandGroup.addCommands(
                    //commands to shoot w/ get shoot command
                );
                break;

            case intake:
                commandGroup.addCommands(
                    //intake from either depot or outpost which depends on scoring position
                );
                break;

            case climb:
                commandGroup.addCommands(
                    //go to tower and climb to l1
                );
                break;

            case pivot:
                commandGroup.addCommands(
                    // commands switch hopper position to go up or down
                    );
                break;
        }
    }
    return commandGroup;

    
}
  }
