package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;

public class Flywheel extends SubsystemBase {

    private final FlywheelIO io; 
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    private void runVelocity(AngularVelocity velocityRadsPerSec) { 
        io.setVelocity(velocityRadsPerSec);
    }

    private void runVoltage(Voltage voltage){
        io.setVoltage(voltage);
    }

    private void stop(){
        io.stop();
    }

    public Command runVoltageCommand(Supplier<Voltage> voltage) {
        return runEnd(() -> runVoltage(voltage.get()), this::stop);
    }

    public Command runFixedVelocityCommand(Supplier<AngularVelocity> velocityRadsPerSec) {
        return runEnd(() -> runVelocity(velocityRadsPerSec.get()), this::stop)
            .withName("Shooter/Flywheel/FixedVelocityCommand/" + velocityRadsPerSec.toString());
    }

    public Command runTargetedCommand() {
        return runEnd(()-> runVelocity(ShooterTrajectoryCalculator.getInstance().getParameters().flywheelVelocity()), this::stop)
            .withName("Shooter/Flywheel/TargetedCommand");
    }

    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Shooter/Flywheel/StopCommand");
    }
}
