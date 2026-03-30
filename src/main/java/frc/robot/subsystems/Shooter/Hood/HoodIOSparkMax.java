package frc.robot.subsystems.Shooter.Hood;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Shooter.Hood.HoodConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Voltage;


public class HoodIOSparkMax implements HoodIO {

    private final SparkMax motor;
    private final RelativeEncoder encoderRelative;
    private final SparkClosedLoopController controller;
    private final ArmFeedforward feedforward;


    public HoodIOSparkMax() {
        motor = new SparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig
            .inverted(HOOD_INVERTED)
            .idleMode(HOOD_IDLE_MODE)
            .smartCurrentLimit(30)
            .voltageCompensation(12.0);
        motorConfig
            .encoder
            .positionConversionFactor(HOOD_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(HOOD_VELOCITY_CONVERSION_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        motorConfig
            .closedLoop
            .pid(HOOD_KP, HOOD_KI, HOOD_KD);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoderRelative = motor.getEncoder();
        controller = motor.getClosedLoopController();
        feedforward = new ArmFeedforward(HOOD_KS, HOOD_KG, HOOD_KV);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodConnected = motor.getLastError() == REVLibError.kOk;
        inputs.hoodAngle = Radians.of(encoderRelative.getPosition());
        inputs.hoodVelocity = RadiansPerSecond.of(encoderRelative.getVelocity());
        inputs.hoodVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()); 
        inputs.hoodCurrentDraw = Amps.of(motor.getOutputCurrent());
        inputs.hoodTemperature = Celsius.of(motor.getMotorTemperature());
    }

    @Override
    public void runVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void runAngle(double angle, double velocity) {
        Logger.recordOutput("Hood/FeedForward Output", feedforward.calculate(angle, velocity));
        controller.setSetpoint(
            angle,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(angle, velocity)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        motor.configure(
            new SparkMaxConfig()
                .apply(new ClosedLoopConfig()
                    .p(kP)
                    .i(kI)
                    .d(kD)),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void setFF(double ks, double kv, double kg) {
        feedforward.setKs(ks);
        feedforward.setKv(kv);
        feedforward.setKg(kg);
    }


}