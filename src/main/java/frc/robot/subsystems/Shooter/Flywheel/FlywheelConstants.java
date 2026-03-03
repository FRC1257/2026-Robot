package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelConstants {

    public static final double FLYWHEEL_KP = 0.0; 
    public static final double FLYWHEEL_KI = 0.0; 
    public static final double FLYWHEEL_KD = 0.0; 

    public static final double FLYWHEEL_KS = 0.55;
    public static final double FLYWHEEL_KV = 0.036;

    public static final double FLYWHEEL_VOLTAGE_TOLERANCE = 0.1;
    public static final int FLYWHEEL_MOTOR_ID = 13;
    public static final int FLYWHEEL_FOLLOWER_MOTOR_ID = 15;

    public static final double SYSID_RAMP_RATE = 0.5;

    public static final double SYSID_STEP_VOLTAGE = 0;

    public static final double SYSID_TIME = 0.5;

    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(500);
}
