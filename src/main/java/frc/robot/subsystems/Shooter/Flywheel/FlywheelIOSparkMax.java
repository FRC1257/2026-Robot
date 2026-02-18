package frc.robot.subsystems.Shooter.Flywheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
            .inverted(false);
        

        // NEEDS TO BE DETERMINED
        flywheelConfig
            .encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);

        flywheelConfig
            .closedLoop
            .p(FlywheelConstants.FLYWHEEL_KP);

        followerConfig = new SparkFlexConfig();

        followerConfig.apply(flywheelConfig);
        followerConfig.follow(motor);


    }

}