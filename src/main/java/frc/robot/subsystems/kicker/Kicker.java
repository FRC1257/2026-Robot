package frc.robot.subsystems.kicker;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Kicker extends SubsystemBase {

    private final KickerIO kickerIO;
    private final KickerIOInputsAutologged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io) {
        this.kickerIO = io;
    }

    @Override
    public void periodic() {
        kickerIO.updateInputs(inputs);
        if(inputs.ballDetected && shooter.getVoltage() > 0){
            kickerIO.setVoltage(4); //probably a different number just guessed here
        }
        else{
            kickerIO.setVoltage(0);
        }
    }

    public Command runVoltageCommand(){
        return run (() -> kickerIO.setVoltage(voltage.get())).withName("Voltage");
    }

    public Command runRPMCommand(){
        return run(() -> kickerIO.setRPM(rpm.get())).withName("RPM");
    }

    public Command stopCommand(){
        return run(() -> kickerIO.stop()).withName("Stop");
    }


}