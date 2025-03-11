package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.Supplier;

/** A command that aligns the robot to a certain field-relative position */
public class AlignToPose extends Command {
  // This command works by using a simple PID loop to move x/y positions and rotate angle
  private PIDController xPidController, yPidController, thetaPidController;
  private double xP, xI, xD;
  private double yP, yI, yD;
  private double thetaP, thetaI, thetaD;
  private Supplier<Pose2d> targetPoseSupplier;
  private Drive drive;

  /**
   * A command that aligns the robot to a certain field-relative position
   *
   * @param drive The (swerve) drivetrain subsystem
   * @param targetPoseSupplier A function returning the desired position (field-relative, using blue
   *     as the origin)
   */
  public AlignToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;

    addRequirements(drive);

    xP = yP = DriveConstants.kTranslationP;
    xI = yI = DriveConstants.kTranslationI;
    xD = yD = DriveConstants.kTranslationD;

    thetaP = DriveConstants.kTurnAngleP;
    thetaI = DriveConstants.kTurnAngleI;
    thetaD = DriveConstants.kTurnAngleD;

    xPidController = new PIDController(xP, xI, xD);
    yPidController = new PIDController(yP, yI, yD);
    thetaPidController = new PIDController(thetaP, thetaI, thetaD);

    thetaPidController.enableContinuousInput(0, 2 * Constants.PI);
  }

  @Override
  public void initialize() {
    // Set the setpoints once at the start of the command
    // Prevents rapidly oscillating movement by guaranteeing only 1 setpoint at a time
    xPidController.setSetpoint(targetPoseSupplier.get().getX());
    yPidController.setSetpoint(targetPoseSupplier.get().getY());
    thetaPidController.setSetpoint(targetPoseSupplier.get().getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    // Field-relative PID calculations for how much to move in x and y directions
    double xOutput =
        xPidController.calculate(currentPose.getX()) * DriveConstants.kMaxSpeedMetersPerSecond;
    double yOutput =
        yPidController.calculate(currentPose.getY()) * DriveConstants.kMaxSpeedMetersPerSecond;

    // Normalize x and y velocity vectors
    // if they want the robot to move faster than it physically can
    double magnitude = Math.sqrt(xOutput * xOutput + yOutput * yOutput);
    if (magnitude > DriveConstants.kMaxSpeedMetersPerSecond) {
      xOutput = xOutput / magnitude * DriveConstants.kMaxSpeedMetersPerSecond;
      yOutput = yOutput / magnitude * DriveConstants.kMaxSpeedMetersPerSecond;
    }

    // PID calculation for how much to turn
    double thetaOutput =
        MathUtil.clamp(
            thetaPidController.calculate(currentPose.getRotation().getRadians())
                * drive.getMaxAngularSpeedRadPerSec(),
            -DriveConstants.kAlignMaxAngularSpeed,
            DriveConstants.kAlignMaxAngularSpeed);

    // Convert field-relative speeds to robot-relative speeds
    ChassisSpeeds driveSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, thetaOutput, drive.getRotation());

    drive.runVelocity(driveSpeeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
