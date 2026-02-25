package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface HopperPivotIO {

  @AutoLog
  public static class HopperPivotIOInputs {
    Voltage pivotVoltage = Volts.of(0.0);
    Angle pivotAngle = Radians.of(0.0);
    AngularVelocity pivotVelocity = RadiansPerSecond.of(0.0); 
  }

  default void updateInputs(HopperPivotIOInputs inputs) {}

  default void runVoltage(Voltage volts) {}

  default void setBreakMode(boolean enabled) {}

  default void stop() {}

}