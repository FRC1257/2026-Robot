package frc.robot.subsystems.Hopper.HopperIntake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class HopperIntake extends SubsystemBase {

  private final HopperIntakeIO io;
  private HopperIntakeIOInputsAutoLogged inputs = new HopperIntakeIOInputsAutoLogged();
    
    
  public HopperIntake(HopperIntakeIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }
    
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("HopperIntake", inputs);
  }
  
  /**
   * Runs the hopper intake at a given voltage. Can be used for both intaking and outtaking fuel by passing in the appropriate voltage.
   * @param voltage the voltage to run the hopper intake at
   * @return a command that runs the hopper intake at the given voltage while it is scheduled
   */

  private Command runVoltage(Supplier<Voltage> voltage) {
    return this.run(() -> io.setVoltage(voltage.get()));
  }

  /**
   * Runs the hopper intake at the voltage specified in {@link HopperIntakeConstants}. This is the default command for the hopper intake and should be used whenever the hopper intake needs to be on.
   * @return a command that runs the hopper intake at the voltage specified in {@link HopperIntakeConstants} while it is scheduled
   */

  public Command runIntake() {
    return runVoltage(() -> HopperIntakeConstants.HOPPER_INTAKE_VOLTAGE).withName("HopperIntake/INTAKE");
  }

  /**
   * Runs the hopper intake in reverse at the voltage specified in {@link HopperIntakeConstants}. This should be used whenever the hopper intake needs to be reversed, such as when unjamming fuel.
   * @return a command that runs the hopper intake in reverse at the voltage specified in {@link HopperIntakeConstants} while it is scheduled
   */
  public Command runOutake() {
    return runVoltage(() -> HopperIntakeConstants.HOPPER_OUTTAKE_VOLTAGE).withName("HopperIntake/OUTTAKE");
  }

  /**
   * Stops the hopper intake by running it at 0 volts.
   * @return a command that stops the hopper intake while it is scheduled
   */

  public Command stopIntake() {
    return runVoltage(() -> Volts.of(0.0)).withName("HopperIntake/STOP");
  }

  /**
   * Stops the hopper intake. This is functionally the same as {@link #stopIntake()}, but is provided for convenience when a command is needed that only stops the hopper intake without any additional functionality.
   * @return a command that stops the hopper intake while it is scheduled
   */
  
  public Command stopCommand() {
    return runOnce(io::stop);
  }

}
