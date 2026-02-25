package frc.robot.subsystems.Shooter.Flywheel;


import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;

public class Flywheel extends SubsystemBase {

    private final FlywheelIO io; 
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
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
            .withName("Shooter/Flywheel/VelocityCommand/" + velocityRadsPerSec.toString());
    }

    /**
     * Runs the flywheel at the velocity specified by the {@link ShooterTrajectoryCalculator} in order to shoot at a target at a given distance.
     * @return a command that runs the flywheel at the velocity specified by the {@link ShooterTrajectoryCalculator} while it is scheduled
     */

    public Command runTargetedCommand() {
        return runEnd(()-> runVelocity(ShooterTrajectoryCalculator.getInstance().getParameters().flywheelVelocity()), this::stop)
            .withName("Shooter/Flywheel/TargetedCommand");
    }

    /**
     * Stops the flywheel. This is functionally the same as {@link #stop()}, but is provided for convenience when a command is needed that only stops the flywheel without any additional functionality.
     * @return a command that stops the flywheel while it is scheduled
     */
    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Shooter/Flywheel/StopCommand");
    }
}
