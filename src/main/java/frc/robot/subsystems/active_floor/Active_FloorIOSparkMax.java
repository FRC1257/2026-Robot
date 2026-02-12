package main.java.frc.robot.subsystems.active_floor;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

public class Active_FloorIOSparkMax implements Active_FloorIO {
    private final CANSparkFlex motor;
    private RelativeEncoder motorEncoder;
    public Active_FloorIOSparkMax() {
        motor = new CANSparkFlex(Active_FloorConstants.Floor_Motor_ID, MotorType.kBrushless);
        //copied from 2025 robot
        config.idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(Neo_Vortex_Current_Limit)
        .inverted(true);
    }
    @Override
    public void updateInputs(Active_FloorIOInputs inputs) {
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.tempCelsius = motor.getMotorTemperature();
    }
    @Override
    public void floor_on(double voltage) {
        motor.setVoltage(voltage);
    }
    @Override
    public void floor_off() {
        motor.setVoltage(0);
    }
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
    @Override
    public void setBrake(boolean brake) {
        //copied from2025 robot
        config.idleMode(brake ? IdleMode.kBreak : IdleMode.kCoast);
        motor.configure (config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}