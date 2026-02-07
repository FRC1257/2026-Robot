package frc.robot.subsystems.kicker;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    public static class KickerIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVoltage = 0.0;
        public double motorCurrent = 0.0;
    }
    public default void updateInputs(KickerIOInputs inputs){}

    public default void setVoltage(double voltage){}
    
    public default double getVoltage(){
        return 0.0;
    }

    public default void setRPM(double RPM){}

    public default double getRPM(){
        return 0.0;
    }
    
    public default void stop(){}
    
}
