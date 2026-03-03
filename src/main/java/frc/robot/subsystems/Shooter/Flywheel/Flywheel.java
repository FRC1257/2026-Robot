package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;
import frc.robot.util.misc.LoggedTunableMeasure;
import frc.robot.util.misc.LoggedTunableNumber;


public class Flywheel extends SubsystemBase {
    
    private static final LoggedTunableNumber Kp = new LoggedTunableNumber("Flywheel/Kp", FlywheelConstants.FLYWHEEL_KP);
    private static final LoggedTunableNumber Ki = new LoggedTunableNumber("Flywheel/Ki", FlywheelConstants.FLYWHEEL_KI);
    private static final LoggedTunableNumber Kd = new LoggedTunableNumber("Flywheel/Kd", FlywheelConstants.FLYWHEEL_KD);

    private static final LoggedTunableNumber Ks = new LoggedTunableNumber("Flywheel/Ks", FlywheelConstants.FLYWHEEL_KS);
    private static final LoggedTunableNumber Kv = new LoggedTunableNumber("Flywheel/Kv", FlywheelConstants.FLYWHEEL_KV);

    private final FlywheelIO io; 
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private SysIdRoutine sysId;


    public Flywheel(FlywheelIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(FlywheelConstants.SYSID_RAMP_RATE),
                Volts.of(FlywheelConstants.SYSID_STEP_VOLTAGE),
                Seconds.of(FlywheelConstants.SYSID_TIME),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volage) -> runVoltage(volage),
                null,
                this));
    }
    

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        if(Kp.hasChanged(hashCode()) || Ki.hasChanged(hashCode()) || Kd.hasChanged(hashCode())) {
            io.setPID(Kp.get(), Ki.get(), Kd.get());
        }

        if(Ks.hasChanged(hashCode()) || Kv.hasChanged(hashCode())) {
            io.setFF(Ks.get(), Kv.get());
        }
        Logger.processInputs("Flywheel", inputs);
    }

    /**
     * Runs the flywheel at a given velocity.
     * @param velocityRadsPerSec the velocity to run the flywheel at in radians per second
     */

    private void runVelocity(AngularVelocity velocityRadsPerSec) { 
        io.setVelocity(velocityRadsPerSec);
    }

    /**
     * Runs the flywheel at a given voltage. This should only be used for testing and tuning purposes.
     * @param voltage the voltage to run the flywheel at in volts
     */

    private void runVoltage(Voltage voltage){
        io.setVoltage(voltage);
    }

    /**
     * Stops the flywheel. 
     */

    private void stop(){
        io.stop();
    }

    /**
     * Runs the flywheel at a given voltage. This should only be used for testing and tuning purposes, as it does not use velocity control and can lead to inconsistent shooting.
     * @param voltage the voltage to run the flywheel at, as a Supplier to allow for dynamic voltages
     * @return a command that runs the flywheel at the given voltage while it is scheduled
     */

    public Command runVoltageCommand(Supplier<Voltage> voltage) {
        return runEnd(() -> runVoltage(voltage.get()), this::stop);
    }

    /**
     * Runs the flywheel at a given velocity. This should be used whenever the flywheel needs to be on, as it will allow for more consistent shooting by using velocity control instead of voltage control.
     * @param velocityRadsPerSec the velocity to run the flywheel at, as a Supplier to allow for dynamic velocities
     * @return a command that runs the flywheel at the given velocity while it is scheduled
     */

    public Command runVelocityCommand(Supplier<AngularVelocity> velocityRadsPerSec) {
        return runEnd(() -> runVelocity(velocityRadsPerSec.get()), this::stop)
            .withName("/Shooter/Flywheel/VelocityCommand/" + velocityRadsPerSec.toString());
    }

    /**
     * Runs the flywheel at the velocity specified by the {@link ShooterTrajectoryCalculator} in order to shoot at a target at a given distance.
     * @return a command that runs the flywheel at the velocity specified by the {@link ShooterTrajectoryCalculator} while it is scheduled
     */

    public Command runTargetedCommand() {
        return runEnd(()-> runVelocity(ShooterTrajectoryCalculator.getInstance().getParameters().flywheelVelocity()), this::stop)
            .withName("/Flywheel/TargetedCommand");
    }

    /**
     * Stops the flywheel. This is functionally the same as {@link #stop()}, but is provided for convenience when a command is needed that only stops the flywheel without any additional functionality.
     * @return a command that stops the flywheel while it is scheduled
     */
    
    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("/Flywheel/StopCommand");
    }

    public Command quasistaticForward(){
        return sysId.quasistatic(Direction.kForward)
            .until(() -> inputs.flywheelAngularVelocity.gte(FlywheelConstants.MAX_VELOCITY));
    }

    public Command quasistaticReverse(){
        return sysId.quasistatic(Direction.kReverse)
            .until(() -> inputs.flywheelAngularVelocity.lte(FlywheelConstants.MAX_VELOCITY.unaryMinus()));
        
    }

    public Command dynamicForward(){
        return sysId.dynamic(Direction.kForward)
            .until(() -> inputs.flywheelAngularVelocity.gte(FlywheelConstants.MAX_VELOCITY));
    }

    public Command dynamicReverse(){
        return sysId.dynamic(Direction.kReverse)
            .until(() -> inputs.flywheelAngularVelocity.lte(FlywheelConstants.MAX_VELOCITY.unaryMinus()));
    }

}
