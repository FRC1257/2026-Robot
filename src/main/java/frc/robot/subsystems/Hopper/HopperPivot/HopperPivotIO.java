package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface HopperPivotIO {

  @AutoLog
  public static class HopperPivotIOInputs {
    boolean leftpivotConnected = false;
    Voltage leftpivotVoltage = Volts.of(0.0);
    Angle leftpivotAngle = Radians.of(0.0);
    AngularVelocity leftpivotVelocity = RadiansPerSecond.of(0.0); 
    Current leftpivotCurrent = Amps.of(0.0);

    boolean rightpivotConnected = false;
    Voltage rightpivotVoltage = Volts.of(0.0);
    Angle rightpivotAngle = Radians.of(0.0);
    AngularVelocity rightpivotVelocity = RadiansPerSecond.of(0.0); 
    Current rightpivotCurrent = Amps.of(0.0);
  }

  default void updateInputs(HopperPivotIOInputs inputs) {}

  default void runVoltage(Voltage volts) {}

  default void runAngle(double angle, double velocity) {}

  default void setPID(double kp, double ki, double kd) {}

  default void setFF(double ks, double kv, double kg) {}

  default void stop() {}

}