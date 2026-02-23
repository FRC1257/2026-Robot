package frc.robot.subsystems.HopperIntake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIntakeIO {

    @AutoLog
    public static class HopperIntakeIOInputs {

    public double velocityRadsPerSec; 
    public double appliedVoltage;
    public double[] currentAmps;
    public double[] tempCelcius;

  }

  public default void updateInputs(HopperIntakeIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}

  public default void setBrake(boolean brake) {}

  public default void stop() {}
}
