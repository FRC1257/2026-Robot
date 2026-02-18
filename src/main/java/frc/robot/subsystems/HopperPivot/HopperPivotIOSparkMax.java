package frc.robot.subsystems.HopperPivot;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class HopperPivotIOSparkMax implements HopperPivotIO {
  // Motor and Encoders
  private SparkMax leftMotor;
  private SparkMax rightMotor;
  
  private SparkMaxConfig leftConfig;
  private SparkMaxConfig rightConfig;
  private final ProfiledPIDController pidController;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

  private SparkAbsoluteEncoder leftEncoder;
  private SparkAbsoluteEncoder rightEncoder;

  //private DigitalInput breakBeam;

  private double setpoint = 0;

  private double kP = HopperPivotConstants.HOPPER_PIVOT_PID_REAL[0],
      kI = HopperPivotConstants.HOPPER_PIVOT_PID_REAL[1],
      kD = HopperPivotConstants.HOPPER_PIVOT_PID_REAL[2];
 

  // These variables are used to find the acceleration of the PID setpoint
  // (change in velocity / change in time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  public HopperPivotIOSparkMax() {
    leftMotor = new SparkMax(HopperPivotConstants.HOPPER_PIVOT_LEFT_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(HopperPivotConstants.HOPPER_PIVOT_RIGHT_ID, MotorType.kBrushless);



    leftEncoder = leftMotor.getAbsoluteEncoder();
    rightEncoder = rightMotor.getAbsoluteEncoder();

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



    
    pidController =
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                HopperPivotConstants.HOPPER_PIVOT_MAX_VELOCITY,
                HopperPivotConstants.HOPPER_PIVOT_MAX_ACCELERATION));


    leftMotor.configure(leftConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;
    rightMotor.configure(rightConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;
    configureFeedForward();
    
   // breakBeam = new DigitalInput(HopperPivotConstants.BREAK_BEAM_CHANNEL);
  }
  
  private void configureFeedForward() {
    setkS(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[0]);
    setkG(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[1]);
    setkV(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[2]);
    setkA(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[3]);
  }

  

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(HopperPivotIOInputs inputs) {
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = leftEncoder.getVelocity();
    inputs.appliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.setpointAngleRads = pidController.getSetpoint().position;
    //inputs.breakBeamBroken = isBreakBeamBroken();

    inputs.currentAmps = new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
    inputs.tempCelsius = new double[] {leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature()};
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("HopperPivot/Desired Voltage", motorVolts);
    leftMotor.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return leftEncoder.getPosition();
  }

  @Override
  public double getAngVelocity() {
    return leftEncoder.getVelocity();
  }

  @Override
  public void setSetpoint(double setpoint) {
    pidController.setGoal(setpoint);
    pidController.reset(getAngle(), getAngVelocity());
    Logger.recordOutput("HopperPivot/Actual Setpoint", pidController.getSetpoint().position);
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = 0, ffOutput = 0;

   


    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    leftMotor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters); 
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE;
  }

 @Override
  public void setkS(double kS) {
    feedforward.setKs(kS);
  }

  @Override
  public void setkG(double kG) {
    feedforward.setKg(kG);
  }

  @Override
  public void setkV(double kV) {
    feedforward.setKv(kV);
  }

  @Override
  public void setkA(double kA) {
    feedforward.setKa(kA);
  }

 

  // @Override
  // public boolean isBreakBeamBroken() {
  //   // return breakBeam.get();
  //   return false;
  // } //we dont have a break beam right?
}