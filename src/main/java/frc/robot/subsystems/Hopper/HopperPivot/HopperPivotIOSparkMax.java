package frc.robot.subsystems.Hopper.HopperPivot;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HopperPivotIOSparkMax implements HopperPivotIO {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    
    private SparkMaxConfig leftConfig;
    private SparkMaxConfig rightConfig;




  //private DigitalInput breakBeam;

  private double setpoint = 0;


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
        .absoluteEncoder
        .zeroCentered(true)
        .zeroOffset(HopperPivotConstants.HOPPER_PIVOT_OFFSET)
        .positionConversionFactor(2 * Constants.PI)
        .velocityConversionFactor(2 * Constants.PI);

    rightConfig = new SparkMaxConfig();
    rightConfig.apply(leftConfig);
    rightConfig.follow(leftMotor);



    



    leftMotor.configure(leftConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;
    rightMotor.configure(rightConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;

    
    }


  
@Override
    public void updateInputs(HopperPivotIOInputs inputs) {
      inputs.pivotVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
      inputs.pivotAngle = leftMotor.getAbsoluteEncoder().getPosition();
      inputs.pivotVelocity = leftMotor.getAbsoluteEncoder().getVelocity();
    inputs.angleRads = getAngle();
    inputs.appliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.setpointAngleRads = setpoint;
    }
@Override
  public void runVoltage(Voltage volts) {

  }
@Override
  public void setBreakMode(boolean enabled) {

  }
@Override
  public void stop() {

  }
}