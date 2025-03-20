// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.ModuleConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public static final int ODOMETRY_FREQUENCY = 250;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        io.setDrivePID(kDrivingP, kDrivingI, kDrivingD);
        io.setDriveFF(kDrivingS, kDrivingV);
        io.setTurnPID(kTurningP, kTurningI, kTurningD);
        io.setTurnFF(kTurningS, kTurningV);
        break;
      case SIM:
        io.setDrivePID(1, 0, 0);
        io.setDriveFF(0.02567, 0.09360);
        io.setTurnPID(5, 0, 0);
        io.setTurnFF(0, 0.12);
        break;
      default:
        io.setDrivePID(0, 0, 0);
        io.setDriveFF(0, 0);
        io.setTurnPID(0, 0, 0);
        io.setTurnFF(0, 0);
        break;
    }
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * kWheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    io.setDriveVelocity(state.speedMetersPerSecond / kWheelRadiusMeters);
    io.setTurnPosition(state.angle.getRadians());

    return state;
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveVoltage(output);
    io.setTurnPosition(0);
  }

  public void runCharacterization(double driveVolts, double angleVolts) {
    // Open loop drive control
    io.setDriveVoltage(driveVolts);
    io.setTurnVoltage(angleVolts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * kWheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * kWheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
