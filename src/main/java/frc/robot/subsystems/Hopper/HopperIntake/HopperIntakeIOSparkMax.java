package frc.robot.subsystems.Hopper.HopperIntake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
        config.smartCurrentLimit(40);
        config.inverted(true);

        config.encoder
            .positionConversionFactor(Math.PI * 2.0) // Convert encoder ticks to radians
            .velocityConversionFactor(Math.PI * 2.0 / 60.0); // Convert encoder ticks per second to radians per second

        motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
        

    }

    @Override
    public void updateInputs(HopperIntakeIOInputs inputs) {
        inputs.intakeVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.intakeVelocity = RadiansPerSecond.of(encoder.getVelocity());
        inputs.intakeCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
        Logger.recordOutput("HopperIntake/Desired Voltage", voltage);
    }

    @Override
    public void setBrake(boolean brake) {
            config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
            motor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

}
