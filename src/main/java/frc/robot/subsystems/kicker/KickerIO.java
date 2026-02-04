package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public double velocity=0; //in RPM
        public double appliedVoltage=0;
        public double current=0; //amps
        public double temperature=0; //celcius
    }
    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default double getVoltage() {
        return 0; //not sure if this is supposed to be here
    }

    public default void setVelocity(double rpm) {};

    public default double getVelocity() {
        return 0;
    }

    public default void setPIDGains(double Kp, double Ki, double Kd) {};
}
