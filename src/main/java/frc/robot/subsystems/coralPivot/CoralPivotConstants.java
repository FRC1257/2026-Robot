package frc.robot.subsystems.coralPivot;

import edu.wpi.first.math.util.Units;

public class CoralPivotConstants {
  // May have to change these constants, same as pivot arm from last year right now

  public static final int CORAL_PIVOT_ID = 15; // Change later

  public static final double[] CORAL_PIVOT_PID_REAL = {3, 0.3, 0};
  public static final double[] CORAL_PIVOT_FEEDFORWARD_REAL = {0.09, 0.43, 0.31973, 0.02};

  public static final double CORAL_PIVOT_PID_TOLERANCE = Units.degreesToRadians(3);

  public static final double CORAL_PIVOT_OFFSET = 0.9081909; // 1.14;

  public static final double CORAL_PIVOT_MAX_ANGLE = Units.degreesToRadians(90);
  public static final double CORAL_PIVOT_MIN_ANGLE = Units.degreesToRadians(-70);

  // deleted the old constants from last years code, this intake angle is from last year still
  public static final double CORAL_PIVOT_STATION_ANGLE = 0.987;
  public static final double CORAL_PIVOT_L1_ANGLE = Units.degreesToRadians(-20);
  public static final double CORAL_PIVOT_L2_L3_ANGLE = -0.1;
  public static final double CORAL_PIVOT_STOW_ANGLE = Units.degreesToRadians(85);
  // Will have to add constants for placing coral

  public static final double SYSID_RAMP_RATE = 0.5;
  public static final double SYSID_STEP_VOLTAGE = 1;
  public static final double SYSID_TIME = 10;

  public static final double CORAL_PIVOT_VOLTAGE_TOLERANCE = 1.0;

  public static final double CORAL_PIVOT_CONTROL_SPEED_FACTOR = 1.0;

  public static final double CORAL_PIVOT_MAX_VELOCITY = 4;
  public static final double CORAL_PIVOT_MAX_ACCELERATION = 13;

  // DIO Channel the absolute encoder is plugged into
  public static final int ABSOLUTE_ENCODER_CHANNEL = 0;

  public static class CoralPivotSimConstants {
    public static final double[] kPivotSimPID = {35, 0, 0};
    public static final double[] kPivotSimFF = {0, 0.57445, 3.9692, 0.0037023};

    public static final double kArmReduction = 200;
    public static final double kArmMass = 10.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(20);
    public static final double kMinAngleRads = Units.degreesToRadians(-70);
    public static final double kMaxAngleRads = Units.degreesToRadians(90);
  }
}
