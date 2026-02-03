package frc.robot.subsystems.Shooter.Flywheel;

import java.util.function.DoubleSupplier;

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
    }

    public Command runFixedVelocityCommand(DoubleSupplier velocity) {
        return runEnd(() -> io.setVelocity(velocity.getAsDouble()), () -> io.stop())
            .withName("Shooter/Flywheel/FixedVelocityCommand/" + velocity.toString());
    }

    public Command runTargetedCommand() {
        return runEnd(()-> io.setVelocity(ShooterTrajectoryCalculator.getInstance().getParameters().flywheelVelocity()), () -> io.stop())
            .withName("Shooter/Flywheel/TargetedCommand");
    }
}
