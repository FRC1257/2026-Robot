package frc.robot.subsystems.Shooter.Hood;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;


public class HoodIOSparkMax implements HoodIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    private final SparkMaxConfig config;


    public HoodIOSparkMax() {
        motor = new SparkMax(HoodConstants.HOOD_MOTOR_ID, SparkMax.MotorType.kBrushless);
        encoder = motor.getEncoder();

        config = new SparkMaxConfig();
        config
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .voltageCompensation(12.0)
            .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
            .inverted(false);

        config
            .encoder
            .positionConversionFactor(2* Math.PI * 1 / 33)
            .velocityConversionFactor((2* Math.PI * 1 / 33) / 60.0);

        motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); 
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodAngle = Radians.of(motor.getEncoder().getPosition());
        inputs.hoodVelocity = RadiansPerSecond.of(motor.getEncoder().getVelocity());
        inputs.hoodVolts = Volts.of(motor.getAppliedOutput() * 12.0); 
    }

    @Override
    public void runVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }


    }