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

    // taking into account 3 positions on each side + neutral zone
    // in front of tower, outpost, or depot
public static enum StartPositions {
    s1,
    s2,
    s3,
    s4,
    s5,
    s6,
    s7,
    s8,
    s9
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




private LoggedDashboardChooser<StartPositions> startChooser;
  private LoggedDashboardChooser<HopperPosition>[] hopperChoosers = new LoggedDashboardChooser[5];
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
    startChooser.addOption("s7", StartPositions.s7);
    startChooser.addOption("s8", StartPositions.s8);
    startChooser.addOption("s9", StartPositions.s9);



    for (int i = 0; i < 5; i++) {



        actionChooser[i] = new LoggedDashboardChooser<>("Scoring Position " + (i + 1));
        actionChooser[i].addDefaultOption("NONE", Actions.NONE);
        for (Actions pos : Actions.values()) {
            if (pos != Actions.NONE) {
                actionChooser[i].addOption(pos.name(), pos);
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
}