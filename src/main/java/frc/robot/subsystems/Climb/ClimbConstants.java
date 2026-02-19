package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {

    public static final double GEAR_RATIO = 100;
    public static final double MASS_CARRIED = 10.0;
    public static final double DRUM_RADIUS = 0.05;
    public static final double MIN_HEIGHT = 0.0;
    public static final double MAX_HEIGHT = 1.5;
    public static final double START_HEIGHT = 0.0;
    

    public static class ClimbIntakeSimConstants {
    public static final double kClimbIntakeP = 0.001;
    public static final double kClimbIntakeI = 0.0;
    public static final double kClimbIntakeD = 0.0;

    public static final double kClimbIntakeGearing = 1.2;
    public static final double kClimbIntakeDrumRadius = 0.03;
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia =
        0.5
            * kCarriageMass
            * kClimbIntakeDrumRadius
            * kClimbIntakeDrumRadius; 
    }

  public static final int CLIMB_INTAKE_MOTOR_ID = 1257;

  public static final Voltage CLIMB_RUN_VOLTAGE = Volts.of(3.0);
}




 
