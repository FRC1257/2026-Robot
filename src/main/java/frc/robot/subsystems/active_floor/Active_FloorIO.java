package frc.robot.subsystems.Active_Floor;
import org.littletonrobotics.junction.AutoLog;
public interface Active_FloorIO {
    @AutoLog
    public class Active_FloorIOInputs {
        //public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        //public double speedSetpoint = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
    }
    
    public default void updateInputs(Active_FloorIOInputs inputs) {}
    public default void floor_on(double voltage) {};
    public default void floor_off() {};
}
