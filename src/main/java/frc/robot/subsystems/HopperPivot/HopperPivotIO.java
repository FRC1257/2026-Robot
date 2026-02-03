package frc.robot.subsystems.HopperPivot;

import org.littletonrobotics.junction.AutoLog;

public interface HopperPivotIO {
  @AutoLog
  public static class HopperPivotIOInputs {
    public double angleRads = 0.0;
    public double angVelocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double setpointAngleRads = 0.0;
    public boolean breakBeamBroken = false;

    public double[] currentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HopperPivotIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double motorVolts) {}

  /** Returns the current distance measurement. */
  public default double getAngle() {
    return 0.0;
  }

  /** Returns the angular velocity of the arm in radians per second */
  public default double getAngVelocity() {
    return 0.0;
  }

  public default void setSetpoint(double setpoint) {}

  /** Go to Setpoint */
  public default void goToSetpoint() {}

  public default void setBrake(boolean brake) {}

  public default boolean atSetpoint() {
    return false;
  }

  public default boolean isBreakBeamBroken() {
    return false;
  }
}