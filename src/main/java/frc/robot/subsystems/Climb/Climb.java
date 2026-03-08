package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.misc.LoggedTunableNumber;

import static frc.robot.subsystems.Climb.ClimbConstants.*;

public class Climb extends SubsystemBase {

    private static final LoggedTunableNumber retractVolts = new LoggedTunableNumber("Climb/retractVolts", RETRACT_VOLTAGE.in(Volts));
    private static final LoggedTunableNumber holdVolts = new LoggedTunableNumber("Climb/holdVolts", HOLD_VOLTAGE.in(Volts));
    private static final LoggedTunableNumber extendVolts = new LoggedTunableNumber("Climb/extendVolts", EXTEND_VOLTAGE.in(Volts));



    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io) {
        this.io = io; 
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    private Command runVoltage(Supplier<Voltage> volts) {
        return run(() -> io.setVoltage(volts.get()));
    }

    public Command retractClimb() {
        return runVoltage(() -> Volts.of(retractVolts.get()));
    }

    public Command holdClimb() {
        return runVoltage(() -> Volts.of(holdVolts.get()));
    }

    public Command extendClimb() {
        return runVoltage(() -> Volts.of(extendVolts.get()));
    }
}
