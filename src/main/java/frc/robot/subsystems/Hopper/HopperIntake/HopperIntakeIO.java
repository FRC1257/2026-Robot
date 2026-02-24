package frc.robot.subsystems.Hopper.HopperIntake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIntakeIO {

    @AutoLog
    public static class HopperIntakeIOInputs {
      public Voltage intakeVoltage = Volts.of(0.0);
      public AngularVelocity intakeVelocity = RadiansPerSecond.of(0.0);

  }

  public default void updateInputs(HopperIntakeIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}

  public default void setBrake(boolean brake) {}

  public default void stop() {}
}
