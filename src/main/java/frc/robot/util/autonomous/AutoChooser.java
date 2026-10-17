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
        RIGHT_TRENCH_OUTPOST,
        RIGHT_TRENCH_FULL_SWEEP
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
        startChooser.addOption("HUMAN_STATION", StartPositions.RIGHT_TRENCH_OUTPOST); 
        startChooser.addOption("RIGHT_TRENCH_SWEEP", StartPositions.RIGHT_TRENCH_FULL_SWEEP);

    }

    // Initial engineering value; validate against the physical pivot before competition.
    private static final double PIVOT_TIMEOUT_SECONDS = 2.0;

    private Command positionThen(boolean trench, Command continuation) {
        return AutoStage.moveThen(
            "Pivot", hopperPivot.runIntakeAngle(),
            trench ? hopperPivot.atTrench() : hopperPivot.atGoal(),
            PIVOT_TIMEOUT_SECONDS, continuation,
            hopperPivot.setPivotVoltage(() -> edu.wpi.first.units.Units.Volts.of(0)));
    }

    public Command getAutoCommand() {
        StartPositions selected = startChooser.get();
        if (selected == null) selected = StartPositions.DEPOT;
        Pose2d start;
        boolean trench;
        Command route;
        switch (selected) {
            case LEFT_TRENCH:
                start = AutoConstants.LEFT_TRENCH_START_POSITION;
                trench = false;
                route = drive.followPathFileCommand("LT_LNZ").deadlineWith(hopperIntake.runIntake())
                    .andThen(drive.followPathFileCommand("LNZ_LS"))
                    .andThen(targetedScore());
                break;
            case RIGHT_TRENCH:
                start = AutoConstants.RIGHT_TRENCH_START_POSITION;
                trench = true;
                route = drive.followPathFileCommand("RT_RNZ").deadlineWith(hopperIntake.runIntake())
                    .andThen(drive.followPathFileCommand("RNZ_RS").deadlineWith(flywheel.runIdle()))
                    .andThen(targetedScore());
                break;
            case RIGHT_TRENCH_OUTPOST:
                start = AutoConstants.RIGHT_TRENCH_START_POSITION;
                trench = true;
                route = drive.followPathFileCommand("RT_RNZ").deadlineWith(hopperIntake.runIntake())
                    .andThen(drive.followPathFileCommand("RNZ_RS"))
                    .andThen(targetedScore().withTimeout(5.0))
                    .andThen(drive.followPathFileCommand("RS_HP"))
                    .andThen(Commands.waitSeconds(3.0))
                    .andThen(drive.followPathFileCommand("HP_RS"))
                    .andThen(targetedScore());
                break;
            case RIGHT_TRENCH_FULL_SWEEP:
                start = AutoConstants.RIGHT_TRENCH_START_POSITION;
                trench = true;
                route = drive.followPathFileCommand("RT_LS").andThen(targetedScore());
                break;
            case DEPOT:
            default:
                start = AutoConstants.DEPOT_START_POSITION;
                trench = false;
                route = drive.followPathFileCommand("DEPOT_START")
                    .andThen(targetedScore().withTimeout(4.0))
                    .andThen(positionThen(false,
                        drive.followPathFileCommand("DEPOT_END").deadlineWith(hopperIntake.runIntake())
                            .andThen(targetedScore())));
                break;
        }
        final Pose2d initialPose = start;
        return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(initialPose)))
            .andThen(positionThen(trench, route));
    }

    private Command targetedScore() { 
        return flywheel.runTargetedCommand(drive::getPose)
                .alongWith(hood.runTargetedCommand(drive::getPose))
                .alongWith(DriveCommands.joystickHubPoint(drive, () -> 0, () -> 0))
                .alongWith(frc.robot.commands.ShotFeed.create(flywheel, hood, kicker, activeFloor, drive::isHubAligned))
                .alongWith(hopperPivot.runAgitate());
    }
}
