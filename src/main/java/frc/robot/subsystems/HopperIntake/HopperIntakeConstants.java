package frc.robot.subsystems.HopperIntake;

public class HopperIntakeConstants {
  public static class HopperIntakeSimConstants {
    public static final double kHopperIntakeP = 0.001;
    public static final double kHopperIntakeI = 0.0;
    public static final double kHopperIntakeD = 0.0;

    public static final double kHopperIntakeGearing = 1.2;
    public static final double kHopperIntakeDrumRadius = 0.03;
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia =
        0.5
            * kCarriageMass
            * kHopperIntakeDrumRadius
            * kHopperIntakeDrumRadius; 
  }

  public static final int HOPPER_INTAKE_MOTOR_ID = 14;

  public static final double HOPPER_INTAKE_IN_SPEED = 0.3;
  public static final double HOPPER_INTAKE_WEAK_IN_SPEED = 0;
  public static final double HOPPER_INTAKE_OUT_SPEED = -0.3;
  public static final double HOPPER_INTAKE_TOLERANCE = 1;
}