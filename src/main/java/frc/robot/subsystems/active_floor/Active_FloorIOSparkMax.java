package frc.robot.subsystems.active_floor;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

public class Active_FloorIOSparkMax implements Active_FloorIO {
    private final CANSparkFlex motor;
    private RelativeEncoder motorEncoder;
    public Active_FloorIOSparkMax() {
        //create motor object with ID and motor type
        motor = new CANSparkFlex(Active_FloorConstants.Floor_Motor_ID, MotorType.kBrushless);
        //get encoder from the motor
        motorEncoder = motor.getEncoder();
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