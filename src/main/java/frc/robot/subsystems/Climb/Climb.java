import frc.robot.subsystems.Climb;

import java.lang.System.Logger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{
    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io) {
        this.climbIO = io;
    }

    @Override
    public void periodic(){
        climbIO.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    private void setVoltage(Voltage voltage) {
        climbIO.setVoltage(voltage);
    }

    public Command runVoltage(Voltage voltage) {
        return run(() -> setVoltage(voltage.get()));
    }

    public Command stopCommand(){
        return run(() -> climbIO.stop()).withName("Climb/Stop");
    }

    public void setBreak(boolean brake) {
        climbIO.setBreak(brake);
    }

}
