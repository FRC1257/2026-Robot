// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.kSlowModeConstant;
import static frc.robot.subsystems.drive.DriveConstants.kTurnAngleD;
import static frc.robot.subsystems.drive.DriveConstants.kTurnAngleI;
import static frc.robot.subsystems.drive.DriveConstants.kTurnAngleP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.drive.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static double slowMode = 1;
  // kSlowModeConstant;

  private static PIDController angleController =
      new PIDController(kTurnAngleP, kTurnAngleI, kTurnAngleD);
  private static LoggedNetworkBoolean shootSide =
      new LoggedNetworkBoolean("/SmartDashboard/ShootSide", false);

  private static Rotation2d lastRotation = new Rotation2d();
  private static double lastTime = Timer.getFPGATimestamp();

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * slowMode, DEADBAND);

          // Normalize magnitude of velocity vector if it is greater than 1
          if (linearMagnitude > 1) {
            linearMagnitude = 1;
          }

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped = getIsFlipped();
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDriveRobotRelative(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble() * slowMode, DEADBAND);

          // Normalize magnitude of velocity vector if it is greater than 1
          if (linearMagnitude > 1) {
            linearMagnitude = 1;
          }

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to robot relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  new Rotation2d()));
        },
        drive);
  }

  /** Drive robot while pointing at a specific point on the field. */
  public static Command joystickReefPoint(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return new InstantCommand(
            () -> {
              lastRotation = drive.getRotation();
            })
        .andThen(
            Commands.run(
                () -> {
                  Pose2d reefPose =
                      new Pose2d(
                          AllianceFlipUtil.apply(FieldConstants.Reef.center), new Rotation2d());

                  // Apply deadband
                  double linearMagnitude =
                      MathUtil.applyDeadband(
                          Math.hypot(
                              xSupplier.getAsDouble() * slowMode,
                              ySupplier.getAsDouble() * slowMode),
                          DEADBAND);
                  Rotation2d linearDirection =
                      new Rotation2d(
                          xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);

                  // Normalize magnitude of velocity vector if it is greater than 1
                  if (linearMagnitude > 1) {
                    linearMagnitude = 1;
                  }

                  Transform2d targetTransform = drive.getPose().minus(reefPose);
                  Rotation2d targetDirection =
                      new Rotation2d(targetTransform.getX(), targetTransform.getY())
                          .plus(new Rotation2d(Math.PI));

                  // Rotation2d deltaDirection = drive.getRotation().minus(targetDirection);

                  Logger.recordOutput("AimAngle", targetDirection);

                  double dtheta = targetDirection.minus(lastRotation).getRadians();
                  if (dtheta > Constants.PI) {
                    dtheta -= 2 * Constants.PI;
                  } else if (dtheta < -Constants.PI) {
                    dtheta += 2 * Constants.PI;
                  }

                  Logger.recordOutput("DTheta", dtheta);

                  // Simple FF calculation of how much to turn the robot based on how the
                  // setpoint is changing
                  // This is corrected by PID
                  double ffOutput = dtheta / (Timer.getFPGATimestamp() - lastTime);
                  lastTime = Timer.getFPGATimestamp();
                  lastRotation = targetDirection;

                  double omega =
                      angleController.calculate(
                          MathUtil.angleModulus(drive.getRotation().getRadians()),
                          MathUtil.angleModulus(targetDirection.getRadians()));

                  // Square values
                  linearMagnitude = linearMagnitude * linearMagnitude;

                  // Calcaulate new linear velocity
                  Translation2d linearVelocity =
                      new Pose2d(new Translation2d(), linearDirection)
                          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                          .getTranslation();

                  // Convert to robot relative speeds & send command
                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                          linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                          MathUtil.clamp(
                              omega * drive.getMaxAngularSpeedRadPerSec() + ffOutput,
                              -DriveConstants.kAlignMaxAngularSpeed,
                              DriveConstants.kAlignMaxAngularSpeed),
                          getIsFlipped()
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));
                },
                drive));
  }

  /** Drive robot while pointing to the closest reef face. */
  // This function checks which face robot is closest to and updates rotation based on that
  // public static Command joystickReefPoint(
  //     Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
  //   return joystickAnglePoint(
  //       drive,
  //       xSupplier,
  //       ySupplier,
  //       () -> {
  //         // Positions of the centers of each reef face
  //         Pose2d[] reefFacePositions = FieldConstants.Reef.centerFaces;

  //         // closestDistance represents the distance to the closest face
  //         // closestFace represents the index of the closest face
  //         double closestDistance = Double.MAX_VALUE;
  //         int closestFace = 0;

  //         // Goes through all reef face positions and checks which one is closest
  //         for (int i = 0; i < 6; i++) {
  //           Pose2d reefPosition = AllianceFlipUtil.apply(reefFacePositions[i]);

  //           Transform2d robotToReefFace = reefPosition.minus(drive.getPose());

  //           if (robotToReefFace.getTranslation().getNorm() < closestDistance) {
  //             closestDistance = robotToReefFace.getTranslation().getNorm();
  //             closestFace = i;
  //           }
  //         }

  //         // Returns desired angle based on which face is closest
  //         Logger.recordOutput("Closest Reef Face", closestFace);
  //         Logger.recordOutput("Closest Reef Distance", closestDistance);
  //         return AllianceFlipUtil.apply(Rotation2d.fromDegrees(-60 * closestFace));
  //       });
  // }

  public static Command joystickStationPoint(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickAnglePoint(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          Pose2d currentPose = AllianceFlipUtil.apply(drive.getPose());
          Rotation2d targetRotation = AllianceFlipUtil.apply(Rotation2d.fromDegrees(-126));
          if (currentPose.getY() > FieldConstants.fieldWidth / 2) {
            targetRotation = AllianceFlipUtil.apply(Rotation2d.fromDegrees(126));
          }
          return targetRotation;
        });
  }

  public static Command joystickProcessorPoint(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickAnglePoint(
        drive, xSupplier, ySupplier, () -> AllianceFlipUtil.apply(Rotation2d.fromDegrees(90)));
  }

  private static boolean getIsFlipped() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  /** Drive robot while pointing at a specific point on the field. */
  public static Command joystickAnglePoint(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> targetDirection) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble() * slowMode, ySupplier.getAsDouble() * slowMode);

          // Normalize magnitude of velocity vector if it is greater than 1
          if (linearMagnitude > 1) {
            linearMagnitude = 1;
          }

          double omega =
              angleController.calculate(
                  MathUtil.angleModulus(drive.getRotation().getRadians()),
                  targetDirection.get().getRadians());

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to robot relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  MathUtil.clamp(
                      omega * drive.getMaxAngularSpeedRadPerSec(),
                      -DriveConstants.kAlignMaxAngularSpeed,
                      DriveConstants.kAlignMaxAngularSpeed),
                  getIsFlipped()
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /** Toggle Slow Mode */
  public static void toggleSlowMode() {
    if (slowMode == 1) {
      slowMode = kSlowModeConstant;
    } else {
      slowMode = 1;
    }
  }

  public static boolean getPivotSideAngle() {
    if (shootSide.get()) {
      return false;
    }
    return true;
  }
}
