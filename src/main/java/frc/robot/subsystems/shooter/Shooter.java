package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
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

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.shooterIO = io;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.recordOutput("Shooter", inputs);
  }

  private void setVoltage(Voltage voltage){
    shooterIO.setVoltage(voltage);
  }

  private Command runVoltage(Supplier<Voltage> voltage) {
    return run(() -> setVoltage(voltage.get()));
  }

  public void setRPM(AngularVelocity rpm) {
    shooterIO.setRPM(rpm);
  }

  public void stop(){
    shooterIO.stop();
  }

  public void setBreak(boolean brake) {
    shooterIO.setBreak(brake);
  }

  public Command runRPMCommand(Supplier<AngularVelocity> rpm) {
    return run(() -> shooterIO.setRPM(rpm.get()));
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}