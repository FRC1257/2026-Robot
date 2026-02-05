package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.shooterIO = io;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
  }

  public Command runVoltageCommand(Supplier<Voltage> voltage) {
    return run(() -> shooterIO.setVoltage(voltage.get())).withName("Voltage");
  }

  public Command runRPMCommand(Supplier<AngularVelocity> rpm) {
    return run(() -> shooterIO.setRPM(rpm.get())).withName("RPM");
  }

  public Command stopCommand() {
    return run(() -> shooterIO.stop()).withName("Stop");
  }
}