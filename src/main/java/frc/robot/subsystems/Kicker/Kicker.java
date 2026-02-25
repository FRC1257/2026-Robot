package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private double goalVelocity = 0.0;

    public Kicker(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
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
            < KickerConstants.KICKER_VELOCITY_TOLERANCE);
    }

}
