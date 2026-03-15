package frc.robot.util.autonomous;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ActiveFloor.ActiveFloor;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntake;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivot;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.drive.AllianceFlipUtil;

public class AutoChooser {

    public static enum StartPositions {
        LEFT_TRENCH,
        DEPOT,
        RIGHT_TRENCH,
        RIGHT_HUMAN_STATION,
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
        startChooser.addDefaultOption("DEPOT", StartPositions.DEPOT);
        startChooser.addOption("LEFT_TRENCH", StartPositions.LEFT_TRENCH);
        startChooser.addOption("RIGHT_TRENCH", StartPositions.RIGHT_TRENCH);
        startChooser.addOption("HUMAN_STATION", StartPositions.RIGHT_HUMAN_STATION);

    }

    public Command getAutoCommand() {
        StartPositions startPosition = startChooser.get();
        switch(startPosition) {
            case LEFT_TRENCH:
                 return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(AutoConstants.LEFT_TRENCH_START_POSITION)))
                            .andThen(hopperPivot.runIntakeAngle().until(hopperPivot.atGoal()))
                            .andThen(drive.followPathFileCommand("LEFT_TRENCH_START").raceWith(hopperIntake.runIntake()))
                            .andThen(drive.followPathFileCommand("LEFT_TRENCH_END"))
                            .andThen(targetedScore());
            case RIGHT_TRENCH:
                 return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(AutoConstants.RIGHT_TRENCH_START_POSITION)))
                            .andThen(hopperPivot.runIntakeAngle().until(hopperPivot.atGoal()))
                            .andThen(drive.followPathFileCommand("RIGHT_TRENCH_START").raceWith(hopperIntake.runIntake()))
                            .andThen(drive.followPathFileCommand("RIGHT_TRENCH_END"))
                            .andThen(targetedScore());
            case DEPOT:
                 return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(AutoConstants.DEPOT_START_POSITION)))
                            .andThen(hopperPivot.runIntakeAngle().until(hopperPivot.atGoal()))
                            .andThen(drive.followPathFileCommand("DEPOT_START"))
                            .andThen(targetedScore().withTimeout(4.0))
                            .andThen(hopperPivot.runIntakeAngle())
                            .andThen(drive.followPathFileCommand("DEPOT_END").raceWith(hopperIntake.runIntake()))
                            .andThen(targetedScore());
            case RIGHT_HUMAN_STATION:
                return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(AutoConstants.RIGHT_TRENCH_START_POSITION)))
                            .andThen(hopperPivot.runIntakeAngle().until(hopperPivot.atGoal()))
                            .andThen(drive.followPathFileCommand("RIGHT_HUMAN_STATION_START"))
                            .andThen(Commands.waitSeconds(2.5))
                            .andThen(drive.followPathFileCommand("RIGHT_HUMAN_STATION_END"))
                            .andThen(targetedScore());
            default:
                 return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(AutoConstants.LEFT_TRENCH_START_POSITION)))
                            .andThen(hopperPivot.runIntakeAngle().until(hopperPivot.atGoal()))
                            .andThen(drive.followPathFileCommand("LEFT_TRENCH_START").raceWith(hopperIntake.runIntake()))
                            .andThen(drive.followPathFileCommand("LEFT_TRENCH_END"))
                            .andThen(targetedScore());
        }
    }

    private Command targetedScore() { 
        return flywheel.runTargetedCommand(drive::getPose)
                .alongWith(hood.runTargetedCommand(drive::getPose))
                .alongWith(DriveCommands.joystickHubPoint(drive, () -> 0, () -> 0))
                .alongWith(Commands.waitUntil(flywheel.isAtGoal().and(hood.isAtGoal()))
                    .andThen(kicker.runIntake()
                    .alongWith(activeFloor.runActiveFloor())
                    .alongWith(hopperPivot.runAgitate())));
    }
}
