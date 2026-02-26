package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class KickerIOSparkMax implements KickerIO {

    private SparkMax kickerMotor = new SparkMax(KickerConstants.KICKER_MOTOR_ID, SparkMax.MotorType.kBrushless);
    private SimpleMotorFeedforward feedforward;

     public KickerIOSparkMax() {
        feedforward = new SimpleMotorFeedforward(KickerConstants.KICKER_KS, KickerConstants.KICKER_KV);

        kickerMotor.configure(
            new SparkMaxConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .voltageCompensation(12.0)
                .smartCurrentLimit(KickerConstants.KICKER_CURRENT_LIMIT)
                .inverted(false)
            .apply(new EncoderConfig()
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0))
            .apply(new ClosedLoopConfig()
                .p(KickerConstants.KICKER_KP)
                .i(KickerConstants.KICKER_KI)
                .d(KickerConstants.KICKER_KD)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.kickerVoltage = Volts.of(kickerMotor.getAppliedOutput() * 12.0);
        inputs.kickerAngularVelocity = RadiansPerSecond.of(kickerMotor.getEncoder().getVelocity());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        kickerMotor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        kickerMotor
            .getClosedLoopController()
                .setSetpoint(
                    velocity.in(RPM),
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    feedforward.calculate(velocity.in(RadiansPerSecond)));
    }

    @Override
    public void stop() {
        kickerMotor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        kickerMotor.configure(
            new SparkMaxConfig()
                .apply(new ClosedLoopConfig()
                    .p(kP)
                    .i(kI)
                    .d(kD)),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double kS, double kV) {
        feedforward = new SimpleMotorFeedforward(kS, kV);
    }

}