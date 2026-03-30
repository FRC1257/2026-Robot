package frc.robot.subsystems.Hopper.HopperIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIntakeIO {

    @AutoLog
    public static class HopperIntakeIOInputs {
      public boolean intakeConnected = false;
      public Voltage intakeVoltage = Volts.of(0.0);
      public AngularVelocity intakeVelocity = RadiansPerSecond.of(0.0);
      public Current intakeCurrent = Amps.of(0.0);
      public Temperature intakeTemperature = Celsius.of(0.0);

      public boolean intakeFollowerConnected = false;
      public Voltage intakeFollowerVoltage = Volts.of(0.0);
      public AngularVelocity intakeFollowerVelocity = RadiansPerSecond.of(0.0);
      public Current intakeFollowerCurrent = Amps.of(0.0);
      public Temperature intakeFollowerTemperature = Celsius.of(0.0);

  }

  public default void updateInputs(HopperIntakeIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}
  
  public default void stop() {}
}
