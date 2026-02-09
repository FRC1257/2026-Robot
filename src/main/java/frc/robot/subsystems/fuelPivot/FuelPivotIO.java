package frc.robot.subsystems.fuelPivot;

import org.littletonrobotics.junction.AutoLog;

public interface FuelPivotIO {
    @AutoLog
    public static class FuelPivotIOInputs {
        public double angleRads = 0.0;
        public double angVelocityRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double velocity = 0.0;
        public double angle = 0.0;
    }

    public default void updateInputs(FuelPivotIOInputs inputs) {}

    public default void setVoltage (double motorVolts) {}

    public default double getAngle() {
        return 0.0;
    }

    public default double getAngleVelocity() {
        return 0.0;
    }

    public default void setSetpoint(double setpoint) {}

    public default void goToSetpoint(){}

    public default void stop() {}

    public default boolean atSetpoint(){
        return false;
    }

    public default void setPIDGains(double kP, double kI, double kD){}

    public default void setFeedForward(double kS, double kG, double kV, double kA){}

    public default double getP() {
        return 0.0;
    }
    
    public default double getI() {
        return 0.0;
    }

    public default double getD() {
        return 0.0;
    }

    public default double getkS() {
        return 0.0;
    }

    public default double getkV() {
        return 0.0;
    }

    public default double getkG() {
        return 0.0;
    }

    public default double getkA () {
        return 0.0;
    }
}
