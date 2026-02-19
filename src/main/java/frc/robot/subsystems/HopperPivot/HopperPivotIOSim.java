package frc.robot.subsystems.HopperPivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController; //
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.HopperPivot.HopperPivotConstants.HopperPivotSimConstants;
import org.littletonrobotics.junction.Logger;

public class HopperPivotIOSim implements HopperPivotIO {


  private final DCMotor m_armGearbox = DCMotor.getNEO(2);


  private final ProfiledPIDController m_controller;
  private ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0, 0);

  private double appliedVoltage = 0;


  double lastSpeed = 0;
  double lastTime = Timer.getFPGATimestamp();



  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          m_armGearbox,
          HopperPivotSimConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              HopperPivotSimConstants.kArmLength,
              HopperPivotSimConstants.kArmMass),
          HopperPivotSimConstants.kArmLength,
          HopperPivotConstants.HOPPER_PIVOT_MIN_ANGLE,
          HopperPivotConstants.HOPPER_PIVOT_MAX_ANGLE,
          true,
          0.1);

  public HopperPivotIOSim() {
    m_controller =
        new ProfiledPIDController(
            HopperPivotSimConstants.kPivotSimPID[0],
            HopperPivotSimConstants.kPivotSimPID[1],
            HopperPivotSimConstants.kPivotSimPID[2],
            new TrapezoidProfile.Constraints(2.45, 2.45));

    m_controller.setTolerance(0.1, 0.05);

    m_feedforward =
        new ArmFeedforward(
            HopperPivotSimConstants.kPivotSimFF[0],
            HopperPivotSimConstants.kPivotSimFF[1],
            HopperPivotSimConstants.kPivotSimFF[2],
            HopperPivotSimConstants.kPivotSimFF[3]);
  }

  @Override
  public void updateInputs(HopperPivotIOInputs inputs) {
    sim.update(0.02);
    inputs.angleRads = getAngle();
    inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.setpointAngleRads = m_controller.getSetpoint().position;
    inputs.appliedVolts = appliedVoltage;
  }

  @Override
  public void setVoltage(double motorVolts) {
    sim.setInputVoltage(motorVolts);
    appliedVoltage = motorVolts;
  }

  @Override
  public void setSetpoint(double setpoint) {
    m_controller.setGoal(setpoint);
    m_controller.reset(getAngle(), getAngVelocity());
  }

  @Override
  public void goToSetpoint() {
    double pidOutput = m_controller.calculate(getAngle());

    double acceleration =
        (m_controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    Logger.recordOutput("HopperPivot/Acceleration", acceleration);
    double ffOutput =
        m_feedforward.calculate(
            m_controller.getSetpoint().position, m_controller.getSetpoint().velocity, acceleration);

    setVoltage(MathUtil.clamp(pidOutput + ffOutput, -12, 12));

    lastSpeed = m_controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }
}