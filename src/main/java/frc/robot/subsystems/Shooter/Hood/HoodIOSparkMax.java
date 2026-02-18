package frc.robot.subsystems.Shooter.Hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Hood.HoodConstants;
import frc.robot.subsystems.HopperPivot.HopperPivotConstants;

public class HoodIOSparkMax implements HoodIO {
     private final SparkFlex motor; 
     private final RelativeEncoder encoder;
     private final ProfiledPIDController controller;

    public HoodIOSparkMax() {
        motor = new SparkFlex(HoodConstants.HOOD_MOTOR_ID, SparkFlex.MotorType.kBrushless);
        encoder = motor.getEncoder();

            controller =
        new ProfiledPIDController(
            HoodConstants.kP,
            HoodConstants.kI,
            HoodConstants.kD,
            new TrapezoidProfile.Constraints(
                HopperPivotConstants.HOPPER_PIVOT_MAX_VELOCITY,
                HopperPivotConstants.HOPPER_PIVOT_MAX_ACCELERATION));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {

        inputs.positionRads = Units.Radians.of(encoder.getPosition());
        inputs.velocityRadsPerSec = Units.RadiansPerSecond.of(encoder.getVelocity());
        inputs.appliedVolts = Units.Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());

    }


    @Override
    public void setAngle(Angle angle) {
            
            double targetPosition = angle.in(Units.Radians);

            controller.setGoal(targetPosition);
    }


    @Override
    public void zero() {
        controller.reset(0.0);
    }
}
