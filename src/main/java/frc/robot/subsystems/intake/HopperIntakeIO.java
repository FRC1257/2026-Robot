package frc.robot.subsystems.HopperIntake;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIntakeIO {

    @AutoLog
    public static class HopperIntakeIOInputs {

    public double velocityRadsPerSec = 0.0;

    public double appliedVoltage = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }
}