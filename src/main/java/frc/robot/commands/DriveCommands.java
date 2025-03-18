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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
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
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Normalize magnitude of velocity vector if it is greater than 1
          if (linearMagnitude > 1) {
            linearMagnitude = 1;
          }

          // Multiply by slow mode factor
          linearMagnitude *= slowMode;
          omega *= slowMode;

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
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Normalize magnitude of velocity vector if it is greater than 1
          if (linearMagnitude > 1) {
            linearMagnitude = 1;
          }

          // Multiply by slow mode factor
          linearMagnitude *= slowMode;
          omega *= slowMode;

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
                          Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                  Rotation2d linearDirection =
                      new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                  // Normalize magnitude of velocity vector if it is greater than 1
                  if (linearMagnitude > 1) {
                    linearMagnitude = 1;
                  }

                  // Multiply by slow mode factor
                  linearMagnitude *= slowMode;

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
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Normalize magnitude of velocity vector if it is greater than 1
          if (linearMagnitude > 1) {
            linearMagnitude = 1;
          }

          // Multiply by slow mode factor
          linearMagnitude *= slowMode;

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

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
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

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
