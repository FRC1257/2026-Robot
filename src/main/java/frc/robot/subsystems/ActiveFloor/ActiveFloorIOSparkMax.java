package frc.robot.subsystems.ActiveFloor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelConstants;
import edu.wpi.first.units.Units;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ActiveFloorIOSparkMax implements ActiveFloorIO {
    private final SparkFlex motor;
    private final SparkFlexConfig config;

    
    public ActiveFloorIOSparkMax() {
        motor = new SparkFlex(ActiveFloorConstants.ACTIVE_FLOOR_MOTOR_ID, SparkFlex.MotorType.kBrushless);
        config = new SparkFlexConfig();

        config
            .smartCurrentLimit(Constants.NEO_VORTEX_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12.0)
            .inverted(false);
        

        // NEEDS TO BE DETERMINED
        config
            .encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;
    }

    @Override
    public void updateInputs(ActiveFloorIOInputs inputs) {
        inputs.activeFloorVoltage = Units.Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setBreakMode(boolean enabled) {
        config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        motor.configure(
            config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
    }

}
