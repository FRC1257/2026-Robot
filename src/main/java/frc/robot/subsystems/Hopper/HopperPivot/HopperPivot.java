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
import org.littletonrobotics.junction.LoggedRobot;
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
import edu.wpi.first.wpilibj2.command.Commands;
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

    private static final LoggedTunableNumber maxVel = new LoggedTunableNumber("HopperPivot/MaxVelocity", HopperPivotConstants.HOPPER_CONSTRAINTS.maxVelocity);
    private static final LoggedTunableNumber maxAccel = new LoggedTunableNumber("HopperPivot/MaxAcceleration", HopperPivotConstants.HOPPER_CONSTRAINTS.maxAcceleration);

    private final HopperPivotIO io;
    private HopperPivotIOInputsAutoLogged inputs = new HopperPivotIOInputsAutoLogged();

    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = null;

    private MechanismLigament2d pivot = new MechanismLigament2d("Hopper Pivot", 0.4, 0, 5, new Color8Bit(Color.kAqua));

    private Angle goalAngle = Radians.of(0.0);

    public HopperPivot(HopperPivotIO io) {
        this.io = io; 
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get()));    
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperPivot", inputs);


        if(Ks.hasChanged(hashCode()) || Kg.hasChanged(hashCode()) || Kv.hasChanged(hashCode())) {
            io.setFF(Ks.get(), Kv.get(), Kg.get());
        }

        if(Kp.hasChanged(hashCode()) || Ki.hasChanged(hashCode()) || Kd.hasChanged(hashCode())) {
            io.setPID(Kp.get(), Ki.get(), Kd.get());
        }

        if(maxVel.hasChanged(hashCode()) || maxAccel.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get()));
        }

        pivot.setAngle(inputs.leftpivotAngle.in(Degrees));

 
    }

    public Command runAngleCommand(Supplier<Angle> angle) {
        return run(() -> {
            goalAngle = angle.get();
            goal = new TrapezoidProfile.State(angle.get().in(Radians),0);
            setpoint = profile.calculate(LoggedRobot.defaultPeriodSecs, setpoint, goal);

            Logger.recordOutput("HopperPivot/setpoint", setpoint.position);
            Logger.recordOutput("HopperPivot/goal", goal.position);

            io.runAngle(setpoint.position, setpoint.velocity);
        }).beforeStarting(() -> {
            goalAngle = angle.get();
            setpoint = new TrapezoidProfile.State(inputs.leftpivotAngle.in(Radians), inputs.leftpivotVelocity.in(RadiansPerSecond));
        })/*.until(isAtGoal())*/;
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

    public Command runIntakeAngle() {
        return runAngleCommand(() -> Radians.of(-0.033244));
    }

    public Command runStowAngle() {
        return runAngleCommand(() -> Radians.of(-1.6));
    }

    public Command runAgitate() {
        return runAngleCommand(() -> Radians.of(-0.5)).withTimeout(0.4)
            .andThen(runAngleCommand(() -> Radians.of(-0.6)).withTimeout(0.4)).repeatedly();
    }


    /**
     * Creates a Trigger that is active when the hopper pivot is at the goal angle within the tolerance specified in {@link HopperPivotConstants}.
     * @return a Trigger that is active when the hopper pivot is at the goal angle within the specified tolerance
     */

    public Trigger atGoal(){
        return new Trigger(() -> inputs.leftpivotAngle.minus(goalAngle).lte(HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE));
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



}