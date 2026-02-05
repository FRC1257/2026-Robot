package frc.robot.subsystems.shooter;

import static frc.robot.Constants.NEO_CURRENT_LIMIT;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;

public class ShooterIOSparkMax implements ShooterIO{
    private SparkFlex motor;
    private RelativeEncoder encoder;
    private PIDController pidController = new PIDController(0, 0, 0);

    public ShooterIOSparkMax() {
        motor = new SparkFlex(ShooterConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();

        config 
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .voltageCompensation(12)
            .smartCurrentLimit(NEO_CURRENT_LIMIT);

        motor.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);

    }   

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRPM = getRPM();
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage(); 
        inputs.motorCurrent = motor.getOutputCurrent();
    }

    @Override
    public double getRPM() {return encoder.getVelocity();}

    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        pidController.setPID(Kp, Ki, Kd);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput("Shooter/Set Voltage", voltage);
    }

    @Override
    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void setRPM(double rpm) {
        double voltage = pidController.calculate(getRPM(), rpm); 
        motor.setVoltage(voltage);
    }
}
