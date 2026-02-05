package frc.robot.subsystems.ActiveFloor;

import org.littletonrobotics.junction.AutoLog;

public interface ActiveFloorIO {
  @AutoLog
  public static class ActiveFloorIOInputs {

    public double velocityRadsPerSec = 0.0;

    public double appliedVoltage = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(ActiveFloorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}


  public default void setBrake(boolean brake) {}
}