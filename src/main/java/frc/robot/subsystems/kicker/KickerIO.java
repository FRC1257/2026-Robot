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
}
