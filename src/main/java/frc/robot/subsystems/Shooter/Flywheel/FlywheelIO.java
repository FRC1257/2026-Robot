package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public AngularVelocity flywheelAngularVelocity;
        public Voltage flywheelVoltage;

    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocityRadsPerSec) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}
}
