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
import static edu.wpi.first.units.Units.RPM;  
import static edu.wpi.first.units.Units.Volts; 
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOSparkMax implements ShooterIO {
    private SparkFlex motor;
    private RelativeEncoder encoder;
    private PIDController pidController = new PIDController(0, 0, 0);

    public ShooterIOSparkMax() {
        motor = new SparkFlex(ShooterConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        SparkFlexConfig config = new SparkFlexConfig();
        config 
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .voltageCompensation(12)
            .smartCurrentLimit(NEO_CURRENT_LIMIT);

        motor.configure(
            config,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters);
    }   

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRPM = getRPM();
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage(); 
        inputs.motorCurrent = motor.getOutputCurrent();
    }

    @Override
    public double getRPM() {
        return encoder.getVelocity();
    }

    @Override
    public void setPIDGains(double Kp, double Ki, double Kd) {
        pidController.setPID(Kp, Ki, Kd);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts)); 
        Logger.recordOutput("Shooter/SetVoltage", voltage.in(Volts));
    }

    @Override
    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void setRPM(AngularVelocity rpm) {
        double targetRPM = rpm.in(RPM);  
        double voltage = pidController.calculate(getRPM(), targetRPM); 
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}