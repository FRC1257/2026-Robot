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
  

  private Command runVoltage(Supplier<Voltage> voltage) {
    return this.run(() -> io.setVoltage(voltage.get()));
  }

  public Command runIntake() {
    return runVoltage(() -> HopperIntakeConstants.HOPPER_INTAKE_VOLTAGE).withName("HopperIntake/INTAKE");
  }

  public Command runOutake() {
    return runVoltage(() -> HopperIntakeConstants.HOPPER_OUTTAKE_VOLTAGE).withName("HopperIntake/OUTTAKE");
  }

  public Command stopIntake() {
    return runVoltage(() -> Volts.of(0.0)).withName("HopperIntake/STOP");
  }

  public Command stopCommand() {
    return runOnce(io::stop);
  }

}
