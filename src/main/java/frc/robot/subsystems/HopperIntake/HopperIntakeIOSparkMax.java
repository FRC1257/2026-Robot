package frc.robot.subsystems.HopperIntake;

import static frc.robot.Constants.NEO_VORTEX_CURRENT_LIMIT;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.Logger;

public class HopperIntakeIOSparkMax implements HopperIntakeIO {
    private SparkFlex motor;
    private RelativeEncoder encoder;

    private SparkFlexConfig config = new SparkFlexConfig();

    public HopperIntakeIOSparkMax() {
        motor = new SparkFlex(HopperIntakeConstants.HOPPER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure motor
        config.idleMode(IdleMode.kBrake);
        config.voltageCompensation(12);
        config.smartCurrentLimit(NEO_VORTEX_CURRENT_LIMIT);
        config.inverted(true);


       

        motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
        

    }

    @Override
    public void updateInputs(HopperIntakeIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
        inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
        inputs.velocityRadsPerSec = encoder.getVelocity();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput("HopperIntake/Desired Voltage", voltage);
    }

    @Override
    public void setBrake(boolean brake) {
            config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
            motor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

}
