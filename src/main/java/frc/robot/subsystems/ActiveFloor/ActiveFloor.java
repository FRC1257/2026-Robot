package frc.robot.subsystems.ActiveFloor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ActiveFloor extends SubsystemBase {
  private final ActiveFloorIO io;
  ActiveFloorIOInputsAutoLogged inputs = new ActiveFloorIOInputsAutoLogged();

  public ActiveFloor(ActiveFloorIO io) {
    this.io = io;
    SmartDashboard.putData(getName(), this);
  }
public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }
  
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ActiveFloor", inputs);
}
public Command runVoltage(DoubleSupplier voltage) {
    return new RunCommand(()->setVoltage(voltage.getAsDouble()), this)
.withName("voltage");
}

//public Command runVoltage(DoubleSupplier voltage) {
 // return runEnd(
   //   () -> setVoltage(voltage.getAsDouble()), // run while scheduled
    //  () -> setVoltage(0.0),                   // stop when finished/interrupted
    //  this)
 //   .withName("ActiveFloor/RunVoltage");
//}
}