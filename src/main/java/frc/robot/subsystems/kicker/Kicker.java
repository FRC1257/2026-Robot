package frc.robot.subsystems.kicker;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

//    private void setVoltage(double voltage) {
//        io.setVoltage(voltage);
//    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Kicker",inputs);
    }

//    private Command runOutake() {
//        return runVoltage(() -> );
//    }
//    public Command stopCommand() {
//        return runOnce(this::stop)
//    }
//
//    private void stop() {
//        io.stop()
//    }

    public Command runKickerCommand(DoubleSupplier voltage) {
        return this.runEnd(
            () -> io.setVoltage(voltage.getAsDouble()),
            () -> io.setVoltage(0.0)

            );
    }
    private void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    private Command runVoltage(DoubleSupplier voltage) {
        return run(() -> setVoltage(voltage.getAsDouble()));
    }
    private Command stop() {
        return run(()->setVoltage(0));
    }
}
