package frc.robot.subsystems.active_floor;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

public class ActiveFloorIOSparkMax implements Active_FloorIO {
    private final CANSparkFlex motor;
    private RelativeEncoder motorEncoder;
    public ActiveFloorIOSparkMax() {
        motor = new CANSparkFlex(ActiveFloorConstants.Floor_Motor_ID, MotorType.kBrushless);
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
}