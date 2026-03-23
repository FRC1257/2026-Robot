package frc.robot.subsystems.Hopper.HopperIntake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class HopperIntakeConstants {
  public static class HopperIntakeSimConstants {
    
    public static final double kHopperIntakeGearing = 1.2;
    public static final double kHopperIntakeDrumRadius = 0.03;
    public static final double kCarriageMass = 0.15; // Mass in Kg
    public static final double kMomentOfInertia =
        0.5
            * kCarriageMass
            * kHopperIntakeDrumRadius
            * kHopperIntakeDrumRadius; 
  }

  public static final int HOPPER_INTAKE_MOTOR_ID = 8;

  public static final Voltage HOPPER_INTAKE_VOLTAGE = Volts.of(-12);
  public static final Voltage HOPPER_OUTTAKE_VOLTAGE = Volts.of(12.0);
  public static final double HOPPER_INTAKE_TOLERANCE = 1;
}