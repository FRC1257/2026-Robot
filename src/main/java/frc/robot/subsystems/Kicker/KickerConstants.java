package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class KickerConstants {

    public static final int KICKER_MOTOR_ID = 16; // Placeholder Value  

    public static final double KICKER_KP = 0.0; // Placeholder Value
    public static final double KICKER_KI = 0.0; // Placeholder Value
    public static final double KICKER_KD = 0.0; // Placeholder Value

    public static final double KICKER_KS = 0.21; // Placeholder Value
    public static final double KICKER_KV = 0.062; // Placeholder Value

    public static final AngularVelocity KICKER_INTAKE_VELOCITY = RadiansPerSecond.of(-50); // Placeholder Value
    public static final AngularVelocity KICKER_OUTTAKE_VELOCITY = RadiansPerSecond.of(0.0); // Placeholder Value
    public static final AngularVelocity KICKER_MAX_VELOCITY = RadiansPerSecond.of(300);

    public static final double KICKER_VELOCITY_TOLERANCE = 0.01; // Placholder Value

    public static final int KICKER_CURRENT_LIMIT = 80;



}
