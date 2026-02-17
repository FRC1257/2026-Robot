package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;
import frc.robot.subsystems.Shooter.Hood.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {

    private final HoodIO io; 
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private Angle goalAngle = Degrees.of(0.0);
    private boolean isHoodZero = false; 

    public Hood(HoodIO io) {
        this.io = io; 
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public Command runFixedAngleCommand(Supplier<Angle> angle) {
        return run(() -> io.setAngle(angle.get()))
            .withName("Shooter/Hood/FixedVelocityCommand/" + angle.toString());
    }

    public Command runTargetedCommand() {
        return run(()-> io.setAngle(ShooterTrajectoryCalculator.getInstance().getParameters().hoodAngle()))
            .withName("Shooter/Hood/TargetedCommand");
    }

    public Command zeroCommand() {
        return runOnce(io::zero);
    }

}
