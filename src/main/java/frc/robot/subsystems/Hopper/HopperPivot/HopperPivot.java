package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.misc.LoggedTunableNumber;

public class HopperPivot extends SubsystemBase {

    private static final LoggedTunableNumber Kp = new LoggedTunableNumber("HopperPivot/Kp", HopperPivotConstants.HOPPER_PIVOT_KP);
    private static final LoggedTunableNumber Ki = new LoggedTunableNumber("HopperPivot/Ki", HopperPivotConstants.HOPPER_PIVOT_KI);
    private static final LoggedTunableNumber Kd = new LoggedTunableNumber("HopperPivot/Kd", HopperPivotConstants.HOPPER_PIVOT_KD);

    private static final LoggedTunableNumber Ks = new LoggedTunableNumber("HopperPivot/Ks", HopperPivotConstants.HOPPER_PIVOT_KS);
    private static final LoggedTunableNumber Kg = new LoggedTunableNumber("HopperPivot/Kg", HopperPivotConstants.HOPPER_PIVOT_KG);
    private static final LoggedTunableNumber Kv = new LoggedTunableNumber("HopperPivot/Kv", HopperPivotConstants.HOPPER_PIVOT_KV);

    private static final LoggedTunableNumber tolerance = new LoggedTunableNumber("HopperPivot/Tolerance", HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE);

    private static final LoggedTunableNumber maxVel = new LoggedTunableNumber("HopperPivot/MaxVelocity", HopperPivotConstants.HOPPER_CONSTRAINTS.maxVelocity);
    private static final LoggedTunableNumber maxAccel = new LoggedTunableNumber("HopperPivot/MaxAcceleration", HopperPivotConstants.HOPPER_CONSTRAINTS.maxAcceleration);

      private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reall?ocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

    private final HopperPivotIO io;
    private HopperPivotIOInputsAutoLogged inputs = new HopperPivotIOInputsAutoLogged();

    private PIDController controller;
    private ArmFeedforward feedforward;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();


  private SysIdRoutine SysId;

    private MechanismLigament2d pivot = new MechanismLigament2d("Hopper Pivot", 0.4, 0, 5, new Color8Bit(Color.kAqua));

    private double goalAngle = 0.0;

    public HopperPivot(HopperPivotIO io) {
        this.io = io; 
        this.controller = new PIDController(Kp.get(), Ki.get(), Kd.get());
        this.feedforward = new ArmFeedforward(Ks.get(), Kg.get(), Kv.get());

        this.controller.setTolerance(tolerance.get());
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get()));

        SmartDashboard.putData(getName(), this);

            SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(HopperPivotConstants.SYSID_RAMP_RATE),
                Volts.of(HopperPivotConstants.SYSID_STEP_VOLTAGE),
                Seconds.of(HopperPivotConstants.SYSID_TIME),
                (state) -> Logger.recordOutput("/HopperPivot/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> io.runVoltage(volts),
                (sysidLog) -> {
                  sysidLog
                      .motor("pivot")
                      .voltage(m_appliedVoltage.mut_replace(inputs.leftpivotVoltage.in(Volts), Volts))
                      .angularPosition(m_angle.mut_replace(inputs.leftpivotAngle.in(Radians), Radians))
                      .angularVelocity(
                          m_velocity.mut_replace(inputs.leftpivotVelocity.in(RadiansPerSecond), RadiansPerSecond));
                },
                this));
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperPivot", inputs);

        if(Kp.hasChanged(hashCode()) || Ki.hasChanged(hashCode()) || Kd.hasChanged(hashCode())) {
            controller.setPID(Kp.get(), Ki.get(), Kd.get());
        }

        if(Ks.hasChanged(hashCode()) || Kg.hasChanged(hashCode()) || Kv.hasChanged(hashCode())) {
            feedforward = new ArmFeedforward(Ks.get(), Kg.get(), Kv.get());
        }

        if(tolerance.hasChanged(hashCode())) {
            controller.setTolerance(tolerance.get());
        }

        if(maxVel.hasChanged(hashCode()) || maxAccel.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get()));
        }

        pivot.setAngle(inputs.leftpivotAngle.in(Degrees));

 
    }

    /**
     * Runs the hopper pivot to a given angle using a PID Controller and feedforward.
     * @param angle the angle to run the hopper pivot to in radians
     */

    private void runAngle(Angle angle) {
        setpointState = 
            profile.calculate(
                0.02,
                setpointState,
                new TrapezoidProfile.State(angle.in(Radians), 0.0));
        
        double volts = 
            controller.calculate(inputs.leftpivotAngle.in(Radians), setpointState.position)
                + feedforward.calculate(
                    setpointState.position, 
                    setpointState.velocity);
        
        io.runVoltage(Volts.of(volts));
    }

    /**
     * Runs the hopper pivot to a given angle using a PID Controller and feedforward. This should be used whenever the hopper pivot needs to move to a specific angle.
     * @param angle the angle to run the hopper pivot to in radians, as a Supplier to allow for dynamic angles
     * @return a command that runs the hopper pivot to the given angle while it is scheduled
     */

    public Command setPivotAngle(Supplier<Angle> angle) {
        return this.run(() -> {
            goalAngle = angle.get().in(Radians);
            runAngle(angle.get());})
        .beforeStarting(() -> {
            profile = new TrapezoidProfile(HopperPivotConstants.HOPPER_CONSTRAINTS);
            setpointState = new TrapezoidProfile.State(inputs.leftpivotAngle.in(Radians), 0.0);
            })
        .withName("Hopper/Pivot/AngleCommand");
    }

    /**
     * Runs the hopper pivot at a given voltage. This should be used whenever the hopper pivot needs to be run at a specific voltage, such as when manually controlling the hopper pivot with a joystick.
     * @param volts the voltage to run the hopper pivot at, as a Supplier to allow for dynamic voltages
     * @return a command that runs the hopper pivot at the given voltage while it is scheduled
     */

    public Command setPivotVoltage(Supplier<Voltage> volts) {
        return this.startEnd(
            () -> io.runVoltage(volts.get()), 
            () -> io.stop()).withName("Hopper/Pivot/VoltageCommand");
    }

    /**
     * Creates a Trigger that is active when the hopper pivot is at the goal angle within the tolerance specified in {@link HopperPivotConstants}.
     * @return a Trigger that is active when the hopper pivot is at the goal angle within the specified tolerance
     */

    public Trigger atGoal(){
        return new Trigger(() -> Math.abs(inputs.leftpivotAngle.in(Radians) - goalAngle) 
            < HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE);
    }

    /**
     *  Appends a MechanismLigament2d to the pivot ligament for visualization purposes.
     * @param mechanism the MechanismLigament2d to append to the pivot ligament
     * @return the appended MechanismLigament2d
     */

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return pivot.append(mechanism);
    }

    /**
     * Gets the pivot ligament for visualization purposes.
     * @return the pivot ligament
     */
    public MechanismLigament2d getPivot() {
        return pivot;
    }

    /**
     * Runs the SysId routine for the hopper pivot in the forward direction until the hopper pivot reaches the maximum angle specified in {@link HopperPivotConstants}.
     * @return a command that runs the SysId routine for the hopper pivot in the forward direction
     */

    public Command quasistaticForward() { 
        return SysId.quasistatic(Direction.kForward)
            .until(() -> inputs.leftpivotAngle.in(Radians) >= HopperPivotConstants.HOPPER_PIVOT_MAX_ANGLE);
    }

    /**
     * Runs the SysId routine for the hopper pivot in the reverse direction until the hopper pivot reaches the minimum angle specified in {@link HopperPivotConstants}.
     * @return a command that runs the SysId routine for the hopper pivot in the reverse direction
     */

    public Command quasistaticReverse() { 
        return SysId.quasistatic(Direction.kReverse)
            .until(() -> inputs.leftpivotAngle.in(Radians) <= HopperPivotConstants.HOPPER_PIVOT_MIN_ANGLE);
    }

    /**
     * Runs the SysId routine for the hopper pivot in the forward direction with a step input until the hopper pivot reaches the maximum angle specified in {@link HopperPivotConstants}.
     * @return a command that runs the SysId routine for the hopper pivot in the forward direction with a step input
     */

    public Command dynamicForward() {
        return SysId.dynamic(Direction.kForward)
            .until(() -> inputs.leftpivotAngle.in(Radians) >= HopperPivotConstants.HOPPER_PIVOT_MAX_ANGLE);
    }

    /**
     * Runs the SysId routine for the hopper pivot in the reverse direction with a step input until the hopper pivot reaches the minimum angle specified in {@link HopperPivotConstants}.
     * @return a command that runs the SysId routine for the hopper pivot in the reverse direction with a step input
     */
    
    public Command dynamicReverse() {
        return SysId.dynamic(Direction.kReverse)
            .until(() -> inputs.leftpivotAngle.in(Radians) <= HopperPivotConstants.HOPPER_PIVOT_MIN_ANGLE);
    }

}