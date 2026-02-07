package frc.robot.subsystems.HopperPivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HopperPivot extends SubsystemBase {
  private final HopperPivotIOInputsAutoLogged inputs = new HopperPivotIOInputsAutoLogged();

  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;

  private LoggedNetworkNumber logkS;
  private LoggedNetworkNumber logkG;
  private LoggedNetworkNumber logkV;
  private LoggedNetworkNumber logkA;

  private LoggedNetworkNumber logActiveP;
  private LoggedNetworkNumber logActiveI;
  private LoggedNetworkNumber logActiveD;

  private LoggedNetworkNumber logActivekS;
  private LoggedNetworkNumber logActivekG;
  private LoggedNetworkNumber logActivekV;
  private LoggedNetworkNumber logActivekA;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reall?ocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

  private double setpoint = 0;

  private final HopperPivotIO io;

  // Create a Mechanism2d visualization of the arm
  private MechanismLigament2d armMechanism = getArmMechanism();

  private SysIdRoutine SysId;

  public static enum State {
    MANUAL,
    PID,
    SYSID
  }

  private State armState = State.MANUAL;
  private double manualSpeed = 0;

  public HopperPivot(HopperPivotIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/D", io.getD());

    logkS = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/kS", io.getkS());
    logkG = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/kG", io.getkG());
    logkV = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/kV", io.getkV());
    logkA = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/kA", io.getkA());

    logActiveP = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/Active P", io.getP());
    logActiveI = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/Active I", io.getI());
    logActiveD = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/Active D", io.getD());

    logActivekS = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/Active kS", io.getkS());
    logActivekG = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/Active kG", io.getkG());
    logActivekV = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/Active kV", io.getkV());
    logActivekA = new LoggedNetworkNumber("/SmartDashboard/HopperPivot/Active kA", io.getkA());

    SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(HopperPivotConstants.SYSID_RAMP_RATE),
                Volts.of(HopperPivotConstants.SYSID_STEP_VOLTAGE),
                Seconds.of(HopperPivotConstants.SYSID_TIME),
                (state) -> Logger.recordOutput("/HopperPivot/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                v -> io.setVoltage(v.in(Volts)),
                (sysidLog) -> {
                  sysidLog
                      .motor("pivot")
                      .voltage(m_appliedVoltage.mut_replace(inputs.appliedVolts, Volts))
                      .angularPosition(m_angle.mut_replace(inputs.angleRads, Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(inputs.angVelocityRadsPerSec, RotationsPerSecond));
                },
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    ;

    armMechanism.setAngle(Units.radiansToDegrees(inputs.angleRads));

    // Move arm based on state
    switch (armState) {
      case MANUAL:
        move(manualSpeed);
        break;
      case PID:
        runPID();
        break;
      default:
        break;
    }

    // Update the PID constants if they have changed
    if (logP.get() != io.getP()) io.setP(logP.get());

    if (logI.get() != io.getI()) io.setI(logI.get());

    if (logD.get() != io.getD()) io.setD(logD.get());

    if (logkS.get() != io.getkS()) io.setkS(logkS.get());

    if (logkG.get() != io.getkG()) io.setkG(logkG.get());

    if (logkV.get() != io.getkV()) io.setkV(logkV.get());

    if (logkA.get() != io.getkA()) io.setkA(logkA.get());

    if (logActiveP.get() != io.getActiveP()) io.setActiveP(logActiveP.get());

    if (logActiveI.get() != io.getActiveI()) io.setActiveI(logActiveI.get());

    if (logActiveD.get() != io.getActiveD()) io.setActiveD(logActiveD.get());

    if (logActivekS.get() != io.getActivekS()) io.setActivekS(logActivekS.get());

    if (logActivekG.get() != io.getActivekG()) io.setActivekG(logActivekG.get());

    if (logActivekV.get() != io.getActivekV()) io.setActivekV(logActivekV.get());

    if (logActivekA.get() != io.getActivekA()) io.setActivekA(logActivekA.get());

    // Log Inputs
    Logger.processInputs("HopperPivot", inputs);

    Logger.recordOutput(
        "HopperPivot/PivotAbsoluteEncoderConnected",
        inputs.angleRads != HopperPivotConstants.HOPPER_PIVOT_OFFSET);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  @AutoLogOutput(key = "HopperPivot/Is Voltage Close")
  public boolean isVoltageClose(double setVoltage) {
    double voltageDifference = Math.abs(setVoltage - inputs.appliedVolts);
    return voltageDifference <= HopperPivotConstants.HOPPER_PIVOT_TOLERANCE;
  }

 

 
  
}