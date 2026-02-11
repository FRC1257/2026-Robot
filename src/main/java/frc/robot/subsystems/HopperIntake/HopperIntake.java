package frc.robot.subsystems.HopperIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class HopperIntake extends SubsystemBase {
    private final HopperIntakeIO io;
    HopperIntakeIOInputsAutoLogged inputs = new HopperIntakeIOInputsAutoLogged();
    
    
    public HopperIntake(HopperIntakeIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }
    
    //we don't need autologoutput for isVoltage closed right???
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperIntake", inputs);
    }

 
 
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setBrake(boolean brake) {
    io.setBrake(brake);
  }



  public Command runVoltage(DoubleSupplier voltage) {
    return new RunCommand(()->setVoltage(voltage.getAsDouble()), this)
.withName("voltage");
  }

  //public Command runVoltage(DoubleSupplier voltage) {
  //return Commands.runEnd(
   //   () -> setVoltage(voltage.getAsDouble()), 
   //   () -> setVoltage(0.0),                   
   //   this)
   // .withName("voltage");
//}


public Command runOuttake(double voltage){ 
    return runVoltage(() -> -voltage);
}

    public Command manualCommand(DoubleSupplier velocitySupplier) {
    return new FunctionalCommand(
        () -> {},
        () -> setVoltage(velocitySupplier.getAsDouble() * 12),
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  
  public Command manualCommand(double velocity) {
    return manualCommand(() -> velocity);
  }


}
