package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface KickerIO {
  
    @AutoLog
    public static class KickerIOInputs {
        public boolean kickerConnected = false;
        public Voltage kickerVoltage = Volts.of(0.0);
        public AngularVelocity kickerAngularVelocity = RadiansPerSecond.of(0.0);
        public Angle kickerAngle = Radians.of(0.0);
        public Current kickerCurrent = Amps.of(0.0);
    }

    public default void updateInputs(KickerIOInputs inputs) {}
    
    public default void setVoltage(Voltage voltage) {}

    public default void setVelocity(AngularVelocity velocity) {}

    public default void stop() {}

    public default void setPID(double kP, double kI, double kD) {}

    public default void setFF(double kS, double kV) {}

    public default void setBreakMode(boolean enabled) {}
}
