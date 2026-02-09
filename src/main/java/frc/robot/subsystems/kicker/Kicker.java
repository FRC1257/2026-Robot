package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Kicker",inputs);

    }

    public Command stop() {
        return run(()->io.setVoltage(0.0));
    }

    public Command runKickerCommand(DoubleSupplier voltage) {
        return this.runEnd(
            () -> io.setVoltage(voltage.getAsDouble()),
            () -> io.setVoltage(0.0)
            
            );
    }
}
