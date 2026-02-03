package frc.robot.subsystems.HopperPivot;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class HopperPivotIOSparkMax implements HopperPivotIO {
  // Motor and Encoders
  private SparkMax pivotMotor;
  private SparkMaxConfig config;
  private final ProfiledPIDController pidController;
  private final ProfiledPIDController pidControllerActive;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  private ArmFeedforward feedforwardActive = new ArmFeedforward(0, 0, 0, 0);

  private SparkAbsoluteEncoder motorEncoder;

  private DigitalInput breakBeam;

  private double setpoint = 0;

  private double kP = HopperPivotConstants.HOPPER_PIVOT_PID_REAL[0],
      kI = HopperPivotConstants.HOPPER_PIVOT_PID_REAL[1],
      kD = HopperPivotConstants.HOPPER_PIVOT_PID_REAL[2];
  private double kActiveP = HopperPivotConstants.HOPPER_PIVOT_PID_REAL_ACTIVE[0],
      kActiveI = HopperPivotConstants.HOPPER_PIVOT_PID_REAL_ACTIVE[1],
      kActiveD = HopperPivotConstants.HOPPER_PIVOT_PID_REAL_ACTIVE[2];

  // These variables are used to find the acceleration of the PID setpoint
  // (change in velocity / change in time = avg acceleration)
  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();

  public HopperPivotIOSparkMax() {
    pivotMotor = new SparkMax(HopperPivotConstants.HOPPER_PIVOT_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT)
        .inverted(true);

    motorEncoder = pivotMotor.getAbsoluteEncoder();

    config
        .absoluteEncoder
        .zeroCentered(true)
        .zeroOffset(HopperPivotConstants.HOPPER_PIVOT_OFFSET)
        .positionConversionFactor(2 * Constants.PI)
        .velocityConversionFactor(2 * Constants.PI);



    pidController =
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                HopperPivotConstants.HOPPER_PIVOT_MAX_VELOCITY,
                HopperPivotConstants.HOPPER_PIVOT_MAX_ACCELERATION));

    pidControllerActive =
        new ProfiledPIDController(
            kActiveP,
            kActiveI,
            kActiveD,
            new TrapezoidProfile.Constraints(
                HopperPivotConstants.HOPPER_PIVOT_MAX_VELOCITY,
                HopperPivotConstants.HOPPER_PIVOT_MAX_ACCELERATION));

    pivotMotor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters); ;
    configureFeedForward();
    configureFeedForwardActive();

    breakBeam = new DigitalInput(HopperPivotConstants.BREAK_BEAM_CHANNEL);
  }
  //needa figure 
  private void configureFeedForward() {
    setkS(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[0]);
    setkG(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[1]);
    setkV(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[2]);
    setkA(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL[3]);
  }

  private void configureFeedForwardActive() {
    setActivekS(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL_ACTIVE[0]);
    setActivekG(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL_ACTIVE[1]);
    setActivekV(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL_ACTIVE[2]);
    setActivekA(HopperPivotConstants.HOPPER_PIVOT_FEEDFORWARD_REAL_ACTIVE[3]);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(HopperPivotIOInputs inputs) {
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = motorEncoder.getVelocity();
    inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.setpointAngleRads = pidController.getSetpoint().position;
    inputs.breakBeamBroken = isBreakBeamBroken();

    inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("AlgaePivot/Desired Voltage", motorVolts);
    pivotMotor.setVoltage(motorVolts);
  }

  /** Returns the current distance measurement. */
  @Override
  public double getAngle() {
    return motorEncoder.getPosition();
  }

  @Override
  public double getAngVelocity() {
    return motorEncoder.getVelocity();
  }

  @Override
  public void setSetpoint(double setpoint) {
    pidController.setGoal(setpoint);
    pidController.reset(getAngle(), getAngVelocity());
    pidControllerActive.setGoal(setpoint);
    pidControllerActive.reset(getAngle(), getAngVelocity());
    Logger.recordOutput("HopperPivot/Actual Setpoint", pidController.getSetpoint().position);
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = 0, ffOutput = 0;

    if (isBreakBeamBroken()) {
      pidOutput = pidControllerActive.calculate(getAngle());

      // change in velocity / change in time = acceleration
      // Acceleration is used to calculate feedforward
      double acceleration =
          (pidControllerActive.getSetpoint().velocity - lastSpeed)
              / (Timer.getFPGATimestamp() - lastTime);

      Logger.recordOutput("HopperPivot/Acceleration", acceleration);

      ffOutput =
          feedforwardActive.calculate(
              pidControllerActive.getSetpoint().position,
              pidControllerActive.getSetpoint().velocity,
              acceleration);

      lastSpeed = pidControllerActive.getSetpoint().velocity;
    } else {
      pidOutput = pidController.calculate(getAngle());

      // change in velocity / change in time = acceleration
      // Acceleration is used to calculate feedforward
      double acceleration =
          (pidController.getSetpoint().velocity - lastSpeed)
              / (Timer.getFPGATimestamp() - lastTime);

      Logger.recordOutput("HopperPivot/Acceleration", acceleration);

      ffOutput =
          feedforward.calculate(
              pidController.getSetpoint().position,
              pidController.getSetpoint().velocity,
              acceleration);

      lastSpeed = pidController.getSetpoint().velocity;

      Logger.recordOutput("HopperPivot/PID output", pidOutput);
      Logger.recordOutput("HopperPivot/FF output", ffOutput);
    }

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotMotor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters); 
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE;
  }

  @Override
  public boolean isBreakBeamBroken() {
    // return breakBeam.get();
    return false;
  }
}