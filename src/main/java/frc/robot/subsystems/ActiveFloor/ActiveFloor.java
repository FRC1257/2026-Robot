package frc.robot.subsystems.ActiveFloor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ActiveFloor extends SubsystemBase {

    private final ActiveFloorIO io;
    private ActiveFloorIOInputsAutoLogged inputs = new ActiveFloorIOInputsAutoLogged();

    private final Debouncer connectedDebouncer = 
        new Debouncer(0.5, DebounceType.kFalling);
    private final Alert disconnected;


    public ActiveFloor(ActiveFloorIO io) {
        this.io = io;

        disconnected = new Alert("ACTIVE FLOOR MOTOR DISCONNECTED", AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        disconnected.set(!connectedDebouncer.calculate(inputs.activeFloorConnected));

        Robot.batteryLogger.reportCurrentUsage(
            getName(),
            inputs.activeFloorCurrent.in(Amps)
        );
    }

    /**
     * Runs the active floor at a given voltage. Can be used for both on and off by passing in the appropriate voltage.
     * @param voltage the voltage to run the active floor at
     * @return a command that runs the active floor at the given voltage while it is scheduled
     */

    private Command runVoltage(Supplier<Voltage> voltage) {
        return run(() -> io.setVoltage(voltage.get()));
    }

    /**
     * Runs the active floor at the voltage specified in ActiveFloorConstants. This is the default command for the active floor and should be used whenever the active floor needs to be on.
     * @return a command that runs the active floor at the voltage specified in ActiveFloorConstants while it is scheduled
     */
    
    public Command runActiveFloor() {
        return runVoltage(() -> ActiveFloorConstants.ACTIVE_FLOOR_VOLTAGE).withName("ActiveFloor/ON");
    }

    public Command stopActiveFloor() {
        return runVoltage(() -> Volts.of(0.0)).withName("ActiveFloor/OFF");
    }
    
}
