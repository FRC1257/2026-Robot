package frc.robot.subsystems.fuelPivot;

import org.littletonrobotics.junction.AutoLog;a

public interface FuelPivotIO {
    public static class FuelPivotIOInputs {
        //fill with inputs plz
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

    public default void setBrake() {}

    public default boolean atSetpoint(){
        return false;
    }

    public default void setP(double p) {}

    public default void setI(double i) {}

    public default void setD(double d) {}

    public default void setkS(double kS) {}

    public default void setkV(double kV) {}

    public default void setkG(double kG) {}

    public default void setkA(double kA) {}

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
