package main.java.frc.robot.subsystems.active_floor;
import org.littletonrobotics.junction.AutoLog;

public interface Active_FloorIO {
    @AutoLog
    public class ActiveFloorIOInputs {
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
    
    public default void updateInputs(ActiveFloorIOInputs inputs) {}
    public default void floor_on(double voltage) {};
    public default void floor_off() {};
    public default void setBrake(boolean brake) {};
}
