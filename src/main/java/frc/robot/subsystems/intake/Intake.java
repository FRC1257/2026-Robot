package frc.robot.subsystems.intake;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
//need setvol,runout,runin, method stop, priv runvolt, command stop
//learn about lambdas, ::


public class HopperIntake extends SubsystemBase {
  private final HopperIntakeIO io;
  HopperIntakeIOInputsAutoLogged inputs = new HopperIntakeIOInputsAutoLogged();

  public HopperIntake(HopperIntakeIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }

public void setVoltage(double voltage) {
    io.setVoltage(voltage);
}

    public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("HopperIntake", inputs);
}

 public void setBrake(boolean brake) {
    io.setBrake(brake);
  }

  private void stop(boolean stop){
    io.stop(stop);
  }

public Command stopIntake(boolean stop){
    return runOnce(this::stop);
}

private Command runVoltage(Supplier<Voltage> voltage){
    return run(() -> setVoltage(voltage.get))
}

public Command runIntake(){
return runVoltage(() -> HopperIntakeConstants.HOPPER_INTAKE_VOLTAGE);
}

public Command runOutake(){
    return runVoltage(() -> HopperIntakeConstants.HOPPER_OUTTAKE_VOLTAGE);
}

}
