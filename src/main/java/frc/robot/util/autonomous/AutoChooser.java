package frc.robot.util.autonomous;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.ActiveFloor.ActiveFloor;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntake;
import frc.robot.subsystems.drive.Drive;

public class AutoChooser {

    public static enum StartPositions {
        LEFT,
        CENTER,
        RIGHT
    }

    private LoggedDashboardChooser<StartPositions> startChooser;

    private Drive drive;
    private ActiveFloor activeFloor;
    private HopperIntake hopperIntake;
}
