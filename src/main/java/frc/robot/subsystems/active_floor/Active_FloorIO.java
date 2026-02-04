package frc.robot.subsystems.active_floor;
import org.littletonrobotics.junction.AutoLog;
public interface active_floorIO {
    @AutoLog
    public static class active_floorIOInputs {
        //public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double speedSetpoint = 0.0;
        public double[] setAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
    }
    public default void updateInputs(active_floorIOInputs inputs) {}
    public default void floor_on(double voltage) {};
    public default void floor_off() {};
    public default void setVoltage(double voltage) {};
}
