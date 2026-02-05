package frc.robot.subsystems.fuelPivot;

public interface FuelPivotIO {
    public static class FuelPivotIOInputs {
        //fill with inputs plz
    }

    public default void undateInputs(FuelPivotIOInputs inputs) {}

    public default void setVoltage (double motorVolts) {}

    public default double getAngle() {}

    public default double getAngleVelocity() {}

    public default void setSetpoint(double setpoint) {}

    public default void goToSetpoint(){}

    public default void setBrake() {}

    public default boolean atSetpoint(){}

    public default void setP(double p) {}

    public default void setI(double i) {}

    public default void setD(double d) {}

    public default void setkS(double kS) {}

    public default void setkV(double kV) {}

    public default void setkG(double kG) {}

    public default void setkA(double kA) {}

    public default double getP() {}
    
    public default double getI() {}

    public default double getD() {}

    public default double getkS() {}

    public default double getkV() {}

    public default double getkG() {}

    public default double getkA () {}
}
