package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.NEO_VORTEX_CURRENT_LIMIT;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Climb.ClimbIO.ClimbIOInputs;

import org.littletonrobotics.junction.Logger;

public class ClimbIOSparkMax implements ClimbIO {
    private SparkFlex motor;
    private RelativeEncoder encoder;

    private SparkFlexConfig config = new SparkFlexConfig();

    public ClimbIOSparkMax() {
        motor = new SparkFlex(ClimbConstants.CLIMB_INTAKE_MOTOR_ID, MotorType.kBrushless);

        config.idleMode(IdleMode.kBrake);
        config.voltageCompensation(12);
        config.smartCurrentLimit(NEO_VORTEX_CURRENT_LIMIT);

        encoder = motor.getEncoder();

        motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
        inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
        inputs.velocityRadsPerSec = encoder.getVelocity(); //error
        }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput("Climb/Desired Voltage", voltage);
    }
}