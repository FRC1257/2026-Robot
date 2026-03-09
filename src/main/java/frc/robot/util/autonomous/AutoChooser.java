package frc.robot.util.autonomous;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.ActiveFloor.ActiveFloor;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntake;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivot;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Hood.Hood;
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
    private HopperPivot hopperPivot;
    private Kicker kicker;
    private Flywheel flywheel;
    private Hood hood;

    public AutoChooser(Drive drive, ActiveFloor activeFloor, HopperIntake hopperIntake, HopperPivot hopperPivot, Kicker kicker, Flywheel flywheel, Hood hood) {
        this.drive = drive; 
        this.activeFloor = activeFloor;
        this.hopperIntake = hopperIntake;
        this.hopperPivot = hopperPivot;
        this.kicker = kicker;
        this.flywheel = flywheel;
        this.hood = hood;

        startChooser = new LoggedDashboardChooser<>("Starting Position");
        startChooser.addDefaultOption("CENTER", StartPositions.CENTER);
        startChooser.addOption("LEFT", StartPositions.LEFT);
        startChooser.addOption("RIGHT", StartPositions.RIGHT);
    }

    public Command getAutoCommand() {
        StartPositions startPosition = startChooser.get();
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        Pose2d startPositionPose2d;

        return commandGroup;
    }
}
