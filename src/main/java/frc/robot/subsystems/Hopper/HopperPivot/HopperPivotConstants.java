package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class HopperPivotConstants {
  public static final double HOPPER_PIVOT_KP = 1.2;
  public static final double HOPPER_PIVOT_KI = 0.0;
  public static final double HOPPER_PIVOT_KD = 0.0;

  public static final double HOPPER_PIVOT_KS = 0.319;
  public static final double HOPPER_PIVOT_KG = 0.18;
  public static final double HOPPER_PIVOT_KV = 0.8;

  public static final Angle HOPPER_PIVOT_PID_TOLERANCE = Radians.of(0.1);

  public static final Angle STOW_ANGLE = Radians.of(0.0);
  public static final Angle INTAKE_ANGLE = Radians.of(1.6);
  public static final Angle TRENCH_ANGLE = Radians.of(1.0);



  public static final Angle HOPPER_PIVOT_TOLERANCE = Degrees.of(2);

  public static final double HOPPER_PIVOT_MAX_VELOCITY = 3;
  public static final double HOPPER_PIVOT_MAX_ACCELERATION = 8;


  public static final TrapezoidProfile.Constraints HOPPER_CONSTRAINTS = new Constraints(8, 5);

public static final int HOPPER_PIVOT_LEFT_ID = 5;

public static final int HOPPER_PIVOT_RIGHT_ID = 6;

  public static class HopperPivotSimConstants {

    public static final double kArmEncoderDistPerPulse = 1 / 4096;

    public static final double kArmReduction = 200;
    public static final double kArmMass = 10.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(20);
    public static final double kArmAngleMin = 0;
    public static final double kArmAngleMax = 1.6;
  }
}
