package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    public static class ShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVoltage = 0.0;
        public double motorCurrent = 0.0;
    }
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(Voltage voltage) {}

    public default double getVoltage() {return 0;}

    public default void setRPM(AngularVelocity rpm) {}

    public default double getRPM() {return 0;}

    public default void setBreak(boolean brake) {}

    public default void setPIDGains(double Kp, double Ki, double Kd) {}

    public default void stop() {}

}