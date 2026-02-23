package frc.robot.subsystems.HopperIntake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

 
  private void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  private void stop() { 
    io.stop();
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  private Command runVoltage(Supplier<Voltage> voltage) {
    return run(() -> setVoltage(voltage.get()));
  }

  public Command runIntake() {
    return runVoltage(() -> HopperIntakeConstants.HOPPER_INTAKE_VOLTAGE);
  }

  public Command runOutake() {
    return runVoltage(() -> HopperIntakeConstants.HOPPER_OUTTAKE_VOLTAGE);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command manualCommand(double voltage) {
    return new FunctionalCommand(()->{}, () -> setVoltage(voltage), (stop) -> setVoltage(0), () -> false, this);
  }



}
