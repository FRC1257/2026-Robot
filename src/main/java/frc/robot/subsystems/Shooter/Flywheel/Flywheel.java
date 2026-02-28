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


public class Flywheel extends SubsystemBase {
      private SysIdRoutine SysId;
    private final FlywheelIO io; 
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

        private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

      public Flywheel(FlywheelIO io) {
        this.io = io;

     SysId =
      new SysIdRoutine(
 
          new SysIdRoutine.Config(
                Volts.per(Second).of(FlywheelConstants.SYSID_RAMP_RATE),
                Volts.of(FlywheelConstants.SYSID_STEP_VOLTAGE),
                Seconds.of(FlywheelConstants.SYSID_TIME),
                (state) -> Logger.recordOutput("/Flywheel/SysIdTestState", state.toString())),
          
            new SysIdRoutine.Mechanism(
                v -> io.setVoltage(v),
                (sysidLog) -> {
                  sysidLog
                      .motor("flywheel")
                      .voltage(m_appliedVoltage.mut_replace(inputs.flywheelVoltage.in(Volts), Volts))
                      .angularVelocity(m_velocity.mut_replace(inputs.flywheelAngularVelocity.in(RotationsPerSecond), RotationsPerSecond));
                
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

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

    public Command quasistaticForward() {

        return SysId.quasistatic(Direction.kForward)
            .until(() -> inputs.flywheelAngularVelocity.in(RotationsPerSecond) >= FlywheelConstants.MAX_VELOCITY.in(RotationsPerSecond));
    }



  public Command quasistaticBack() {

    return SysId.quasistatic(Direction.kReverse)
        .until(() ->inputs.flywheelAngularVelocity.in(RotationsPerSecond) <= -FlywheelConstants.MAX_VELOCITY.in(RotationsPerSecond));
  }

  public Command dynamicForward() {
    return SysId.dynamic(Direction.kForward)
        .until(() -> inputs.flywheelAngularVelocity.in(RotationsPerSecond) >= FlywheelConstants.MAX_VELOCITY.in(RotationsPerSecond));
  }

  public Command dynamicBack() {

    return SysId.dynamic(Direction.kReverse)
        .until(() -> inputs.flywheelAngularVelocity.in(RotationsPerSecond) <= -FlywheelConstants.MAX_VELOCITY.in(RotationsPerSecond));
  }

}
