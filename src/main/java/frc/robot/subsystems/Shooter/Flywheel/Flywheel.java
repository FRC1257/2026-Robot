package frc.robot.subsystems.Shooter.Flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;

public class Flywheel extends SubsystemBase {

    private final FlywheelIO io; 
    private final FlywheelIOInputs inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;

    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    @AutoLogOutput(key = "AlgaeIntake/Close")
    public boolean isVoltageClose(double setVoltage) {
        return Math.abs(setVoltage - inputs.flywheelVoltage) <= FlywheelConstants.FLYWHEEL_VOLTAGE_TOLERANCE;
    }

    private void runVelocity(double velocityRadsPerSec) { 
        io.setVelocity(velocityRadsPerSec);
    }

    private void stop(){
        io.stop();
    }

    public Command runFixedVelocityCommand(DoubleSupplier velocityRadsPerSec) {
        return runEnd(() -> runVelocity(velocityRadsPerSec.getAsDouble()), this::stop)
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
