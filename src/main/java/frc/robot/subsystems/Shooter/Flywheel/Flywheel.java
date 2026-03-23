package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;
import frc.robot.util.misc.LoggedTunableNumber;


public class Flywheel extends SubsystemBase {
    
    private static final LoggedTunableNumber Kp = new LoggedTunableNumber("Flywheel/Kp", FlywheelConstants.FLYWHEEL_KP);
    private static final LoggedTunableNumber Ki = new LoggedTunableNumber("Flywheel/Ki", FlywheelConstants.FLYWHEEL_KI);
    private static final LoggedTunableNumber Kd = new LoggedTunableNumber("Flywheel/Kd", FlywheelConstants.FLYWHEEL_KD);

    private static final LoggedTunableNumber Ks = new LoggedTunableNumber("Flywheel/Ks", FlywheelConstants.FLYWHEEL_KS);
    private static final LoggedTunableNumber Kv = new LoggedTunableNumber("Flywheel/Kv", FlywheelConstants.FLYWHEEL_KV);

    private static final LoggedTunableNumber hubVelocityRads = new LoggedTunableNumber("Flywheel/hubVelocityRads", 250);

    private final FlywheelIO io; 
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private final Debouncer leaderMotorConnectedDebounder = 
        new Debouncer(0.5, DebounceType.kFalling);
    
    private final Debouncer followerMotorConnectedDebouncer =
        new Debouncer(0.5, DebounceType.kFalling);

    private final Alert leaderDisconnected;
    private final Alert followerDisconnected;

    @AutoLogOutput private AngularVelocity goalVelocity = RadiansPerSecond.of(0.0);

    public Flywheel(FlywheelIO io) {
        this.io = io;

        leaderDisconnected = new Alert("FLYWHEEL LEADER MOTOR DISCONNECTED", AlertType.kError);
        followerDisconnected = new Alert("FLYWHEEL FOLLOWER MOTOR DISCONNECTED", AlertType.kError);

    }
    

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);

        if(Kp.hasChanged(hashCode()) || Ki.hasChanged(hashCode()) || Kd.hasChanged(hashCode())) {
            io.setPID(Kp.get(), Ki.get(), Kd.get());
        }

        if(Ks.hasChanged(hashCode()) || Kv.hasChanged(hashCode())) {
            io.setFF(Ks.get(), Kv.get());
        }

        leaderDisconnected.set(!leaderMotorConnectedDebounder.calculate(inputs.flywheelLeaderConnected));
        followerDisconnected.set(!followerMotorConnectedDebouncer.calculate(inputs.flywheelFollowerConnected));


        Robot.batteryLogger.reportCurrentUsage(
            "Flywheel",
            inputs.flywheelLeaderCurrent.in(Amps),
            inputs.flywheelFollowerCurrent.in(Amps)
        );
    }

    /**
     * Gets the current flywheel velocity based off the latest periodic loop
     * @return current Angular Velocity
     */
    public AngularVelocity getAngularVelocity() {
        return inputs.flywheelLeaderAngularVelocity;
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
        goalVelocity = RadiansPerSecond.of(0.0);
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
     * Runs the flywheel at a fixed angular velocity defined by {@link hubVelocityRads}
     * @return a command that runs the flywheel at the previously mentioned fixed angular velocity
     */

    public Command runHubVelocity() {
        return runVelocityCommand(() -> RadiansPerSecond.of(hubVelocityRads.get()));
    }

    /**
     * Runs the flywheel at a given velocity. This should be used whenever the flywheel needs to be on, as it will allow for more consistent shooting by using velocity control instead of voltage control.
     * @param velocityRadsPerSec the velocity to run the flywheel at, as a Supplier to allow for dynamic velocities
     * @return a command that runs the flywheel at the given velocity while it is scheduled
     */

    private Command runVelocityCommand(Supplier<AngularVelocity> velocityRadsPerSec) {
        return runEnd(() -> {
                goalVelocity = velocityRadsPerSec.get();
                runVelocity(velocityRadsPerSec.get());
            },
            this::stop).beforeStarting(() -> {
                goalVelocity = velocityRadsPerSec.get();
            })
        .withName("/Shooter/Flywheel/VelocityCommand/" + velocityRadsPerSec.toString());
    }

    /**
     * Runs the flywheel at the velocity specified by the {@link ShooterTrajectoryCalculator} in order to shoot at a target at a given distance.
     * @return a command that runs the flywheel at the velocity specified by the {@link ShooterTrajectoryCalculator} while it is scheduled
     */

    public Command runTargetedCommand(Supplier<Pose2d> robotPose) {
        return runVelocityCommand(() -> ShooterTrajectoryCalculator.getInstance().getStaticParameters(robotPose).flywheelVelocity());
    }

    /**
     * Stops the flywheel. This is functionally the same as {@link #stop()}, but is provided for convenience when a command is needed that only stops the flywheel without any additional functionality.
     * @return a command that stops the flywheel while it is scheduled
     */
    
    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("/Flywheel/StopCommand");
    }

    @AutoLogOutput(key="Shooter/Flywheel/isAtGoal")
    public Trigger isAtGoal() {
        return new Trigger(() -> inputs.flywheelLeaderAngularVelocity.isNear(goalVelocity, FlywheelConstants.FLYWHEEL_VELOCITY_TOLERANCE));
    }

}
