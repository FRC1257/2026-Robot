package frc.robot.subsystems.Climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Climb.ClimbConstants.*;

public class ClimbIOSparkMax implements ClimbIO {
    private final SparkMax motor; 
    private final SparkMaxConfig config;

    public ClimbIOSparkMax() {
        motor = new SparkMax(CLIMB_MOTOR_ID, MotorType.kBrushless);
        config = new SparkMaxConfig();

        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .inverted(false);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) { 
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput()*motor.getBusVoltage());
        inputs.currentAmps = Amps.of(motor.getOutputCurrent());
        inputs.temperatureCelcius = Celsius.of(motor.getMotorTemperature());
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }
}
