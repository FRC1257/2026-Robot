package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

public class ElevatorConstants {
  // top height 50 inches (measured from bottom of elevator frame)
  // bottom height 12.375 inches (measured from bottom of frame)
  public static final double ELEVATOR_MAX_HEIGHT = 0.91; // not including chasis
  public static final double ELEVATOR_MIN_HEIGHT = 0.005;

  // SysID constants
  public static final double SYSID_RAMP_RATE = 1;
  public static final double SYSID_STEP_VOLTAGE = 4;
  public static final double SYSID_TIME = 10;

  public static final double ELEVATOR_VOLTAGE_TOLERANCE = 1;

  public static int LEFT_MOTOR_ID = 10;
  public static int RIGHT_MOTOR_ID = 5;
  public static IdleMode MOTOR_DEFAULT_IDLE_MODE = IdleMode.kBrake;
  /** Used for converting angular displacement into linear displacement */
  public static double DRUM_RADIUS_METERS = 0.02236;
  /** Gear ratio of the elevator motors */
  public static double GEAR_RATIO = 1.0 / 20.0;

  /** Final position conversion factor based on drum radius and gear ratio */
  public static double POSITION_CONVERSION_FACTOR =
      4 * Constants.PI * DRUM_RADIUS_METERS * GEAR_RATIO;

  public static final double ELEVATOR_MASS_KG = 5;

  // Max velocity and acceleration of the elevator, in m/s and m/s^2
  public static final double MAX_VELOCITY = 1;
  public static final double MAX_ACCELERATION = 4;

  // Digital input channels
  public static final int ABSOLUTE_ENCODER_CHANNEL = 1;
  public static final int LIMIT_SWITCH_CHANNEL = 9;

  // Height setpoints for elevator
  public static final double ELEVATOR_STATION_HEIGHT = 0.3;
  public static final double ELEVATOR_L1_HEIGHT = 0.4;
  public static final double ELEVATOR_L2_HEIGHT = 0.6;
  public static final double ELEVATOR_L3_HEIGHT = 0.7;

  // PID Constants
  public static final double[] ELEVATOR_REAL_PID = {3, 0, 0};
  public static final double[] ELEVATOR_REAL_FF = {0.2354, 0.16, 8.7, 0.17};

  /** Tolerance used when checking if the elevator is at the setpoint */
  public static double SETPOINT_TOLERANCE_METERS = 0.005;

  public static class ElevatorSimConstants {
    public static final double[] ELEVATOR_SIM_PID = {300, 0, 0};
    public static final double[] ELEVATOR_SIM_FF = {1.4841E-06, 0.12642, 17.748, 1.5689E-05};
    // Convert from encoder steps to meters

    public static final double GEAR_RATIO_SIM = 20;
    public static final double ELEVATOR_STARTING_HEIGHT = 0.2;

    // 4096 pulses per revolution
    // (2pi radians / 4096) * gear ratio
    public static final double ENCODER_DIST_PER_PULSE =
        2 * Math.PI / 4096 * DRUM_RADIUS_METERS * GEAR_RATIO;
    // public static final int kMotorPort = 2;

  }
}
