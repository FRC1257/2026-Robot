// Made just to change spark max because commit didn't work on a different laptop and I don't want
// merging issues

package frc.robot.subsystems.algaePivot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class AlgaePivotIOSparkMax implements AlgaePivotIO {
  // Motor and Encoders
  private SparkMax pivotMotor;
  private SparkMaxConfig config;
  private final SparkClosedLoopController pidController;
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  private ArmFeedforward feedforwardActive = new ArmFeedforward(0, 0, 0, 0);

  private SparkAbsoluteEncoder motorEncoder;

  private DigitalInput breakBeam;

  private double setpoint = 0;

  private double kP = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL[0],
      kI = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL[1],
      kD = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL[2];
  private double kActiveP = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL_ACTIVE[0],
      kActiveI = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL_ACTIVE[1],
      kActiveD = AlgaePivotConstants.ALGAE_PIVOT_PID_REAL_ACTIVE[2];

  public AlgaePivotIOSparkMax() {
    pivotMotor = new SparkMax(AlgaePivotConstants.ALGAE_PIVOT_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

    motorEncoder = pivotMotor.getAbsoluteEncoder();

    config
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .zeroCentered(true)
        .zeroOffset(AlgaePivotConstants.ALGAE_PIVOT_OFFSET)
        .positionConversionFactor(2 * Constants.PI)
        .velocityConversionFactor(2 * Constants.PI / 60.0)
        .startPulseUs(1)
        .endPulseUs(1024);

    // absoluteEncoder.reset();
    // make sure the pivot starts at the bottom position every time
    // absoluteEncoder.reset();

    pidController = pivotMotor.getClosedLoopController();

    config
        .closedLoop
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
        .pid(kActiveP, kActiveI, kActiveD, ClosedLoopSlot.kSlot1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config
        .closedLoop
        .maxMotion
        .maxVelocity(AlgaePivotConstants.ALGAE_PIVOT_MAX_VELOCITY, ClosedLoopSlot.kSlot0)
        .maxAcceleration(AlgaePivotConstants.ALGAE_PIVOT_MAX_ACCELERATION, ClosedLoopSlot.kSlot0)
        .maxVelocity(AlgaePivotConstants.ALGAE_PIVOT_MAX_VELOCITY, ClosedLoopSlot.kSlot1)
        .maxAcceleration(AlgaePivotConstants.ALGAE_PIVOT_MAX_ACCELERATION, ClosedLoopSlot.kSlot1);

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configureFeedForward();
    configureFeedForwardActive();

    breakBeam = new DigitalInput(AlgaePivotConstants.BREAK_BEAM_CHANNEL);
  }

  private void configureFeedForward() {
    setkS(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[0]);
    setkG(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[1]);
    setkV(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[2]);
    setkA(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL[3]);
  }

  private void configureFeedForwardActive() {
    setActivekS(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[0]);
    setActivekG(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[1]);
    setActivekV(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[2]);
    setActivekA(AlgaePivotConstants.ALGAE_PIVOT_FEEDFORWARD_REAL_ACTIVE[3]);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = motorEncoder.getVelocity();
    inputs.appliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.setpointAngleRads = setpoint;
    inputs.breakBeamBroken = isBreakBeamBroken();

    inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double motorVolts) {
    Logger.recordOutput("AlgaePivot/AppliedVolts", motorVolts);
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

  /** Go to Setpoint */
  @Override
  public void goToSetpoint(double setpoint) {

    double feedforwardOutput = 0;

    if (isBreakBeamBroken()) {
      feedforwardOutput = feedforwardActive.calculate(getAngle(), 0);
      pidController.setReference(
          setpoint,
          ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot1,
          feedforwardOutput);
    } else {
      feedforwardOutput = feedforward.calculate(getAngle(), 0);
      pidController.setReference(
          setpoint,
          ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0,
          feedforwardOutput);
    }

    Logger.recordOutput("AlgaePivot/FeedforwardOutput", feedforwardOutput);
  }

  @Override
  public void setBrake(boolean brake) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(getAngle() - setpoint) < AlgaePivotConstants.ALGAE_PIVOT_PID_TOLERANCE;
  }

  @Override
  public boolean isBreakBeamBroken() {
    return breakBeam.get();
  }

  @Override
  public void setP(double p) {
    config.closedLoop.p(p, ClosedLoopSlot.kSlot0);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kP = p;
  }

  @Override
  public void setI(double i) {
    config.closedLoop.i(i, ClosedLoopSlot.kSlot0);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kI = i;
  }

  @Override
  public void setD(double d) {
    config.closedLoop.d(d, ClosedLoopSlot.kSlot0);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kD = d;
  }

  @Override
  public void setkS(double kS) {
    feedforward =
        new ArmFeedforward(kS, feedforward.getKg(), feedforward.getKv(), feedforward.getKa());
  }

  @Override
  public void setkG(double kG) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), kG, feedforward.getKv(), feedforward.getKa());
  }

  @Override
  public void setkV(double kV) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), feedforward.getKg(), kV, feedforward.getKa());
  }

  @Override
  public void setkA(double kA) {
    feedforward =
        new ArmFeedforward(feedforward.getKs(), feedforward.getKg(), feedforward.getKv(), kA);
  }

  @Override
  public double getkS() {
    return feedforward.getKs();
  }

  @Override
  public double getkG() {
    return feedforward.getKg();
  }

  @Override
  public double getkV() {
    return feedforward.getKv();
  }

  @Override
  public double getkA() {
    return feedforward.getKa();
  }

  @Override
  public double getP() {
    return kP;
  }

  @Override
  public double getI() {
    return kI;
  }

  @Override
  public double getD() {
    return kD;
  }

  @Override
  public void setActiveP(double p) {
    config.closedLoop.p(p, ClosedLoopSlot.kSlot1);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kActiveP = p;
  }

  @Override
  public void setActiveI(double i) {
    config.closedLoop.i(i, ClosedLoopSlot.kSlot1);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kActiveI = i;
  }

  @Override
  public void setActiveD(double d) {
    config.closedLoop.d(d, ClosedLoopSlot.kSlot1);
    pivotMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    kActiveD = d;
  }

  @Override
  public void setActivekS(double kS) {
    feedforwardActive =
        new ArmFeedforward(
            kS, feedforwardActive.getKg(), feedforwardActive.getKv(), feedforwardActive.getKa());
  }

  @Override
  public void setActivekG(double kG) {
    feedforwardActive =
        new ArmFeedforward(
            feedforwardActive.getKs(), kG, feedforwardActive.getKv(), feedforwardActive.getKa());
  }

  @Override
  public void setActivekV(double kV) {
    feedforwardActive =
        new ArmFeedforward(
            feedforwardActive.getKs(), feedforwardActive.getKg(), kV, feedforwardActive.getKa());
  }

  @Override
  public void setActivekA(double kA) {
    feedforwardActive =
        new ArmFeedforward(
            feedforwardActive.getKs(), feedforwardActive.getKg(), feedforwardActive.getKv(), kA);
  }

  @Override
  public double getActivekS() {
    return feedforwardActive.getKs();
  }

  @Override
  public double getActivekG() {
    return feedforwardActive.getKg();
  }

  @Override
  public double getActivekV() {
    return feedforwardActive.getKv();
  }

  @Override
  public double getActivekA() {
    return feedforwardActive.getKa();
  }

  @Override
  public double getActiveP() {
    return kActiveP;
  }

  @Override
  public double getActiveI() {
    return kActiveI;
  }

  @Override
  public double getActiveD() {
    return kActiveD;
  }
}
