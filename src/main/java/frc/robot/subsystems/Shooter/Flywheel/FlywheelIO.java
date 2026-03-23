package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public boolean flywheelLeaderConnected = false; 
        public Angle flywheelLeaderAngle = Radians.of(0.0);
        public AngularVelocity flywheelLeaderAngularVelocity = RadiansPerSecond.of(0.0);
        public Voltage flywheelLeaderVoltage = Volts.of(0.0);
        public Current flywheelLeaderCurrent = Amps.of(0.0);
        public Temperature flywheelLeaderTemperature = Celsius.of(0.0);

        public boolean flywheelFollowerConnected = false;
        public Angle flywheelFollowerAngle = Radians.of(0.0);
        public AngularVelocity flywheelFollowerAngularVelocity = RadiansPerSecond.of(0.0);
        public Voltage flywheelFollowerVoltage = Volts.of(0.0);
        public Current flywheelFollowerCurrent = Amps.of(0.0);
        public Temperature flywheelFollowerTemperature = Celsius.of(0.0);

    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocityRadsPerSec) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}

    public default void setPID(double kp, double ki, double kd) {}

    public default void setFF(double ks, double kv) {}
}
