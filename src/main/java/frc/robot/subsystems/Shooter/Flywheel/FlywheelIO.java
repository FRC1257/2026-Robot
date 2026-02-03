package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public double flywheelPositionRotations = 0; 
        public double flywheelAngularVelocity = 0; 
        public double flywheelVoltage = 0;
        public double flywheelOutputCurrent = 0; 
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setVelocity(double velocity) {}

    public default void stop() {}
}
