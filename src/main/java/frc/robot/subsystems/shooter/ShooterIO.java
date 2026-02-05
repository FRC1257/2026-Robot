package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    public static class ShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVoltage = 0.0;
        public double motorCurrent = 0.0;
    }
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default double getVoltage() {return 0;}

    public default void setRPM(double rpm) {}

    public default double getRPM() {return 0;}

    public default void setPIDGains(double Kp, double Ki, double Kd) {}

    public default void stop() {}
}