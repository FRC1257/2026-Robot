package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private double goalVelocity = 0.0;

    public Trigger atGoalVelocityTrigger = new Trigger(this::atGoalVelocity);

    public Kicker(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    public Command runVoltageCommand(Supplier<Voltage> voltage) {
        return runEnd(() -> io.setVoltage(voltage.get()), io::stop)
            .withName(getName() + "/RunVoltageCommand");
    }

    public Command runVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> {
            goalVelocity = velocity.get().in(RadiansPerSecond);
            io.setVelocity(velocity.get());
        }, io::stop)
            .withName(getName() + "/RunVelocityCommand");
    }
    
    public Command stopCommand() {
        return runOnce(io::stop)
            .withName(getName() + "/StopCommand");
    }

    private boolean atGoalVelocity() {
        return Math.abs(inputs.kickerAngularVelocity.in(RadiansPerSecond) - goalVelocity) 
            < KickerConstants.KICKER_VELOCITY_TOLERANCE;
    }



    
}
