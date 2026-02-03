package frc.robot.subsystems.Shooter;

public class ShooterTrajectoryCalculator {

    private static ShooterTrajectoryCalculator instance; 

    public static ShooterTrajectoryCalculator getInstance() {
        if (instance == null) instance = new ShooterTrajectoryCalculator();
        return instance;
    }

    public record ShooterTrajectoryParameters(
        boolean isValid, 
        double flywheelVelocity,
        double hoodAngle,
        double hoodVelocity
    ) {}

    private ShooterTrajectoryParameters latestParameters = null; 

    public ShooterTrajectoryParameters getParameters() {
        if(latestParameters != null) return latestParameters;

        // NEED TO DO ALL THE CALCULATIONS STILL

        return latestParameters;
    }
}
