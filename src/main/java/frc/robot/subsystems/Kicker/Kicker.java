package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelConstants;
import frc.robot.util.misc.LoggedTunableNumber;

public class Kicker extends SubsystemBase {

    private static final LoggedTunableNumber Kp = new LoggedTunableNumber("Kicker/Kp", KickerConstants.KICKER_KP);
    private static final LoggedTunableNumber Ki = new LoggedTunableNumber("Kicker/Ki", KickerConstants.KICKER_KI);
    private static final LoggedTunableNumber Kd = new LoggedTunableNumber("Kicker/Kd", KickerConstants.KICKER_KD);
    
    private static final LoggedTunableNumber Ks = new LoggedTunableNumber("Kicker/Ks", KickerConstants.KICKER_KS);
    private static final LoggedTunableNumber Kv = new LoggedTunableNumber("Kicker/Kv", KickerConstants.KICKER_KV);

    private static final LoggedTunableNumber tolerance = new LoggedTunableNumber("Kicker/Tolerance", KickerConstants.KICKER_VELOCITY_TOLERANCE);

    private static final LoggedTunableNumber kickerIntakeVelocity = new LoggedTunableNumber("Kicker/IntakeVelocity", KickerConstants.KICKER_INTAKE_VELOCITY.magnitude()); 
    private static final LoggedTunableNumber kickerOuttakeVelocity = new LoggedTunableNumber("Kicker/OuttakeVelocity", KickerConstants.KICKER_OUTTAKE_VELOCITY.magnitude());


    private SysIdRoutine sysId; 

    private final KickerIO io;
    private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private double goalVelocity = 0.0;

    public Kicker(KickerIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Kicker/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volage) -> io.setVoltage(volage),
                null,
                this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        if(Kp.hasChanged(hashCode()) || Ki.hasChanged(hashCode()) || Kd.hasChanged(hashCode())) {
            io.setPID(Kp.get(), Ki.get(), Kd.get());
        }

        if(Ks.hasChanged(hashCode()) || Kv.hasChanged(hashCode())) {
            io.setFF(Ks.get(), Kv.get());
        }
    }

    /**
     * Runs the kicker at a given voltage. 
     * @param voltage the voltage to run the kicker at, as a Supplier to allow for dynamic voltages
     * @return a command that runs the kicker at the given voltage while it is scheduled
     */

    public Command runVoltageCommand(Supplier<Voltage> voltage) {
        return runEnd(() -> io.setVoltage(voltage.get()), io::stop)
            .withName(getName() + "/RunVoltageCommand");
    }

    /**
     * Runs the kicker at a given velocity. This should be used whenever the kicker needs to be on, as it will allow for more consistent shooting by using velocity control instead of voltage control.
     * @param velocity the velocity to run the kicker at, as a Supplier to allow for dynamic velocities
     * @return a command that runs the kicker at the given velocity while it is scheduled
     */

    public Command runVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> {
            goalVelocity = velocity.get().in(RadiansPerSecond);
            io.setVelocity(velocity.get());
        }, io::stop)
            .withName(getName() + "/RunVelocityCommand");
    }

    public Command runStatic() {
        return runVoltageCommand(() -> Volts.of(Ks.get()));
    }
    
    /**
     * Runs the kicker at the intake velocity specified in {@link KickerConstants}. This should be used whenever the kicker needs to be intaking, as it will allow for more consistent intaking by using velocity control instead of voltage control.
     * @return a command that runs the kicker at the intake velocity while it is scheduled
     */

    public Command runIntake() {
        return runVelocityCommand(() -> RadiansPerSecond.of(kickerIntakeVelocity.get()))
            .withName(getName() + "/IntakeCommand");
    }

    /**
     * Runs the kicker at the outtake velocity specified in {@link KickerConstants}. This should be used whenever the kicker needs to be outtaking, as it will allow for more consistent outtaking by using velocity control instead of voltage control.
     * @return a command that runs the kicker at the outtake velocity while it is scheduled
     */

    public Command runOuttake() {
        return runVelocityCommand(() -> RadiansPerSecond.of(kickerOuttakeVelocity.get()))
            .withName(getName() + "/OuttakeCommand");
    }
    
    /**
     * Stops the kicker. 
     * @return a command that stops the kicker when it is scheduled
     */

    public Command stopCommand() {
        return runOnce(io::stop)
            .withName(getName() + "/StopCommand");
    }

    /**
     * A Trigger that returns true when the kicker is at the goal velocity within a certain tolerance specified in {@link KickerConstants}. This can be used to determine when the kicker is ready to shoot.
     * @return a Trigger that is active when the kicker is at the goal velocity within the tolerance specified in {@link KickerConstants}
     */

    public Trigger atGoalVelocity() {
        return new Trigger(() -> Math.abs(inputs.kickerAngularVelocity.in(RadiansPerSecond) - goalVelocity) 
            < tolerance.get());
    }

        public Command quasistaticForward(){
        return sysId.quasistatic(Direction.kForward)
            .until(() -> inputs.kickerAngularVelocity.gte(KickerConstants.KICKER_MAX_VELOCITY));
    }

    public Command quasistaticReverse(){
        return sysId.quasistatic(Direction.kReverse)
            .until(() -> inputs.kickerAngularVelocity.lte(KickerConstants.KICKER_MAX_VELOCITY.unaryMinus()));
        
    }

    public Command dynamicForward(){
        return sysId.dynamic(Direction.kForward)
            .until(() -> inputs.kickerAngularVelocity.gte(KickerConstants.KICKER_MAX_VELOCITY));
    }

    public Command dynamicReverse(){
        return sysId.dynamic(Direction.kReverse)
            .until(() -> inputs.kickerAngularVelocity.lte(KickerConstants.KICKER_MAX_VELOCITY.unaryMinus()));
    }

}
