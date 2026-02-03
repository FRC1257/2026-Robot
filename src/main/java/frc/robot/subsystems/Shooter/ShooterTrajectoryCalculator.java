package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterTrajectoryCalculator {

    private static ShooterTrajectoryCalculator instance; 

    public static ShooterTrajectoryCalculator getInstance() {
        if (instance == null) instance = new ShooterTrajectoryCalculator();
        return instance;
    }

    public record ShooterTrajectoryParameters(
        boolean isValid, 
        AngularVelocity flywheelVelocity,
        Angle hoodAngle
    ) {}

    private ShooterTrajectoryParameters latestParameters = null; 

    public ShooterTrajectoryParameters getParameters() {
        if(latestParameters != null) return latestParameters;

        // NEED TO DO ALL THE CALCULATIONS STILL

        return latestParameters;
    }

    public void resetParameters() {
        latestParameters = null;
    }
}
