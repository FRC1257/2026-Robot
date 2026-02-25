package frc.robot.subsystems.Hopper.HopperPivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class HopperPivotConstants {
  // May have to change these constants, same as pivot arm from last year right now

  public static final int HOPPER_PIVOT_ID = 5; // Change later

  public static final double HOPPER_PIVOT_GEARING = 1.0 / 16.0;

  public static final double HOPPER_PIVOT_KP = 8.0;
  public static final double HOPPER_PIVOT_KI = 0.0;
  public static final double HOPPER_PIVOT_KD = 0.0;

  public static final double HOPPER_PIVOT_KS = 0.0;
  public static final double HOPPER_PIVOT_KG = 4.0;
  public static final double HOPPER_PIVOT_KV = 0.0;



  public static final double HOPPER_PIVOT_PID_TOLERANCE = Units.degreesToRadians(1);
  public static final double HOPPER_PIVOT_PID_VELOCITY_TOLERANCE = 0.5;

  public static final double HOPPER_PIVOT_OFFSET = 0.2299176; // 1.14;

  public static final double PIVOT_MAX_PID_TIME = 3;

  public static final double HOPPER_PIVOT_MAX_ANGLE = 1.95;
  public static final double HOPPER_PIVOT_MIN_ANGLE = Units.degreesToRadians(-20);

  public static final double HOPPER_PIVOT_DOWN_ANGLE = Units.degreesToRadians(40);
  public static final double HOPPER_PIVOT_STOW_ANGLE = Units.degreesToRadians(105);
  public static final double HOPPER_PIVOT_AUTO_INTAKE_ANGLE = Units.degreesToRadians(45);
  public static final double HOPPER_PIVOT_PROCESSOR_ANGLE = Units.degreesToRadians(55);
  

  public static final double SYSID_RAMP_RATE = 0.5;
  public static final double SYSID_STEP_VOLTAGE = 1.0;
  public static final double SYSID_TIME = 10;
  public static final double HOPPER_PIVOT_TOLERANCE = 1.0;

  public static final double HOPPER_PIVOT_CONTROL_SPEED_FACTOR = 1.0;

  public static final double HOPPER_PIVOT_MAX_VELOCITY = 3;
  public static final double HOPPER_PIVOT_MAX_ACCELERATION = 8;

  public static final int BREAK_BEAM_CHANNEL = 0;
  public static final int ABSOLUTE_ENCODER_CHANNEL = 1;

  public static final TrapezoidProfile.Constraints HOPPER_CONSTRAINTS = new Constraints(8, 3);

  public static class HopperPivotSimConstants {
    public static final double[] kPivotSimPID = {15, 0, 0, 0};
    public static final double[] kPivotSimFF = {0, 0.574, 0, 0};

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmSetpointDegrees = Units.degreesToRadians(75.0);

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 1 / 4096;

    public static final double kArmReduction = 200;
    public static final double kArmMass = 10.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(20);
    public static final double kArmAngleMin = Units.degreesToRadians(0);
    public static final double kArmAngleMax = Units.degreesToRadians(100);
  }
}
