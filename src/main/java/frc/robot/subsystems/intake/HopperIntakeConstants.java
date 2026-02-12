package frc.robot.subsystems.intake;

public class HopperIntakeConstants {
  public static class HopperIntakeSimConstants {
    // public static final double kAlgaeIntakeP = 0.001;
    // public static final double kAlgaeIntakeI = 0.0;
    // public static final double kAlgaeIntakeD = 0.0;

    public static final double kHopperIntakeGearing = 3;
    public static final double kHopperIntakeDrumRadius = 0.03;
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia =
        0.5
            * kCarriageMass
            * kHopperIntakeDrumRadius
            * kHopperIntakeDrumRadius; 
  }

  public static final int HOPPER_INTAKE_MOTOR_ID = 0;
  public static final int HOPPER_INTAKE_VOLTAGE = 12;
   public static final int HOPPER_OUTTAKE_VOLTAGE = -0;
  

}