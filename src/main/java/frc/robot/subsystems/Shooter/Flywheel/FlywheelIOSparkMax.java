package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FlywheelIOSparkMax implements FlywheelIO {

    private final SparkFlex motor; 
    private final SparkFlex followerMotor; 

    private final RelativeEncoder encoder; 
    private final RelativeEncoder followerEncoder;

    private final SparkFlexConfig flywheelConfig;
    private final SparkFlexConfig followerConfig;

    private final SparkClosedLoopController controller; 
    private final SimpleMotorFeedforward feedforward;

    public FlywheelIOSparkMax() {

        motor = new SparkFlex(FlywheelConstants.FLYWHEEL_MOTOR_ID, SparkFlex.MotorType.kBrushless);
        followerMotor = new SparkFlex(FlywheelConstants.FLYWHEEL_FOLLOWER_MOTOR_ID, SparkFlex.MotorType.kBrushless);

        encoder = motor.getEncoder();
        followerEncoder = followerMotor.getEncoder();


        flywheelConfig = new SparkFlexConfig();

        flywheelConfig
            .smartCurrentLimit(Constants.NEO_VORTEX_CURRENT_LIMIT)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12.0)
            .inverted(true);
        

        // NEEDS TO BE DETERMINED
        flywheelConfig
            .encoder
            .positionConversionFactor(Math.PI*2*32/34)
            .velocityConversionFactor(((Math.PI*2)*32/34) / 60)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        flywheelConfig
            .closedLoop
            .p(FlywheelConstants.FLYWHEEL_KP);

        followerConfig = new SparkFlexConfig();

        followerConfig.apply(flywheelConfig);
        followerConfig.follow(motor, true);


        controller = motor.getClosedLoopController();
        feedforward = new SimpleMotorFeedforward(FlywheelConstants.FLYWHEEL_KS, FlywheelConstants.FLYWHEEL_KV);

        motor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.flywheelAngularVelocity = RadiansPerSecond.of(encoder.getVelocity());
        inputs.flywheelVoltage = Volts.of(motor.getAppliedOutput()*12);

    }

    @Override
    public void setVelocity(AngularVelocity velocityRadsPerSec) {
        double feedforwardVolts = feedforward.calculate(velocityRadsPerSec.magnitude());
        controller.setSetpoint(velocityRadsPerSec.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforwardVolts);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();

    }

}