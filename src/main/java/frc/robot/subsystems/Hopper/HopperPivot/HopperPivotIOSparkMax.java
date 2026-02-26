package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class HopperPivotIOSparkMax implements HopperPivotIO {
  private SparkMax leftMotor;
  private SparkMax rightMotor;
    
  private SparkMaxConfig leftConfig;
  private SparkMaxConfig rightConfig;

  public HopperPivotIOSparkMax() {
    leftMotor = new SparkMax(HopperPivotConstants.HOPPER_PIVOT_LEFT_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(HopperPivotConstants.HOPPER_PIVOT_RIGHT_ID, MotorType.kBrushless);

    leftConfig = new SparkMaxConfig();

    leftConfig
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
        .inverted(true);

    leftConfig
            .encoder
            .positionConversionFactor(2* Math.PI)
            .velocityConversionFactor(2* Math.PI / 60.0);

    rightConfig = new SparkMaxConfig();
    rightConfig.apply(leftConfig);
    rightConfig.follow(leftMotor);

    leftMotor.configure(leftConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;
    rightMotor.configure(rightConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;

    
    }


  
@Override
    public void updateInputs(HopperPivotIOInputs inputs) {
        inputs.leftpivotVelocity = RadiansPerSecond.of(leftMotor.getEncoder().getVelocity());
        inputs.leftpivotVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
        inputs.rightpivotVelocity = RadiansPerSecond.of(rightMotor.getEncoder().getVelocity());
        inputs.rightpivotVoltage = Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
        inputs.leftpivotAngle = Radians.of(leftMotor.getEncoder().getPosition());
        inputs.rightpivotAngle = Radians.of(rightMotor.getEncoder().getPosition());
    }
@Override
  public void runVoltage(Voltage volts) {
    leftMotor.setVoltage(volts);
  }
@Override
  public void setBreakMode(boolean enabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    leftMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
@Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}