package frc.robot.subsystems.Hopper.HopperIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.NEO_VORTEX_CURRENT_LIMIT;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.Logger;

public class HopperIntakeIOSparkMax implements HopperIntakeIO {
    private SparkFlex motor;
    private SparkFlex followerMotor;

    private RelativeEncoder encoder;
    private RelativeEncoder followerEncoder;

    public HopperIntakeIOSparkMax() {
        motor = new SparkFlex(HopperIntakeConstants.HOPPER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new SparkFlex(HopperIntakeConstants.HOPPER_INTAKE_FOLLOWER_MOTOR_ID, MotorType.kBrushless);

        encoder = motor.getEncoder();
        followerEncoder = followerMotor.getEncoder();

        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();


        // Configure motor
        config.idleMode(IdleMode.kCoast);
        config.voltageCompensation(12);
        config.smartCurrentLimit(60);
        config.inverted(true);

        config.encoder
            .positionConversionFactor(Math.PI * 2.0) // Convert encoder ticks to radians
            .velocityConversionFactor(Math.PI * 2.0 / 60.0); // Convert encoder ticks per second to radians per second
        
        followerConfig.apply(config);
        followerConfig.follow(motor, true);
        
        motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
        followerMotor.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 


    }

    @Override
    public void updateInputs(HopperIntakeIOInputs inputs) {
        inputs.intakeConnected = motor.getLastError() == REVLibError.kOk;
        inputs.intakeVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.intakeVelocity = RadiansPerSecond.of(encoder.getVelocity());
        inputs.intakeCurrent = Amps.of(motor.getOutputCurrent());
        inputs.intakeTemperature = Celsius.of(motor.getMotorTemperature());

        inputs.intakeFollowerConnected = followerMotor.getLastError() == REVLibError.kOk;
        inputs.intakeFollowerVoltage = Volts.of(followerMotor.getAppliedOutput() * followerMotor.getBusVoltage());
        inputs.intakeFollowerVelocity = RadiansPerSecond.of(followerEncoder.getVelocity());
        inputs.intakeFollowerCurrent = Amps.of(followerMotor.getOutputCurrent());
        inputs.intakeFollowerTemperature = Celsius.of(followerMotor.getMotorTemperature());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput("HopperIntake/Desired Voltage", voltage);
    }

}
