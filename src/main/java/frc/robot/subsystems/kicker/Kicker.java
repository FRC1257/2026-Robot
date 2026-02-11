package frc.robot.subsystems.kicker;

import java.lang.System.Logger;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {

    private final KickerIO kickerIO;
    private final KickerIOInputsAutologged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io) {
        this.kickerIO = io;
    }

    @Override
    public void periodic() {
        kickerIO.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);
    }
    


    public Command runRPMCommand(DoubleSupplier rpm){
        return run(() -> kickerIO.setRPM(rpm.getAsDouble())).withName("RPM");
    }

   // public Command runRPMCommand(DoubleSupplier rpm) {

  //return runEnd(
    //  () -> kickerIO.setRPM(rpm.getAsDouble()),
    //  () -> kickerIO.stop(),
    //  this)
    //  .withName("Kicker/RunRPM");
//}

    public Command runVoltage(DoubleSupplier voltage) {
    return new StartEndCommand(
        () -> kickerIO.setVoltage(voltage.getAsDouble()),
        () -> kickerIO.setVoltage(0.0),
        this).withName("Kicker/ RunVoltage");
}


    public Command stopCommand(){
        return run(() -> kickerIO.stop()).withName("Stop");
    }


}