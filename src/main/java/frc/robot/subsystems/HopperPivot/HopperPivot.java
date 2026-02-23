package frc.robot.subsystems.HopperPivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperPivot extends SubsystemBase {

    private final HopperPivotIO io;
    private HopperPivotIOInputsAutoLogged inputs = new HopperPivotIOInputsAutoLogged();

    private Angle goalAngle = Degrees.of(0.0);

    public HopperPivot(HopperPivotIO io) {
        this.io = io; 
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperPivot", inputs);
    }

    @AutoLogOutput
    public boolean atGoal(){
        return Math.abs(inputs.pivotAngle.in(Degrees) - goalAngle.in(Degrees)) 
            < HopperPivotConstants.HOPPER_PIVOT_TOLERANCE;
    }

    public Command setPivotAngle(Supplier<Angle> angle) {
        return this.run(() -> {
            goalAngle = angle.get();
            io.runAngle(angle.get());}
        ).withName("Hopper/Pivot/AngleCommand");
    }

    public Command setPivotVoltage(Supplier<Voltage> volts) {
        return this.startEnd(
            () -> io.runVoltage(volts.get()), 
            () -> io.stop()).withName("Hopper/Pivot/VoltageCommand");
    }
}