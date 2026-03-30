package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class KickerConstants {

    public static final int KICKER_MOTOR_ID = 16; // Placeholder Value  

    public static final double KICKER_KP = 0.0; // Placeholder Value
    public static final double KICKER_KI = 0.0; // Placeholder Value
    public static final double KICKER_KD = 0.0; // Placeholder Value

    public static final double KICKER_KS = 0.21; // Placeholder Value
    public static final double KICKER_KV = 0.062; // Placeholder Value

    public static final AngularVelocity KICKER_INTAKE_VELOCITY = RadiansPerSecond.of(-75); // Placeholder Value
    public static final AngularVelocity KICKER_OUTTAKE_VELOCITY = RadiansPerSecond.of(0.0); // Placeholder Value
    public static final AngularVelocity KICKER_MAX_VELOCITY = RadiansPerSecond.of(300);
    public static final AngularVelocity KICKER_JAMMED_VELOCITY = RadiansPerSecond.of(10.0);

    public static final Current KICKER_JAMMED_CURRENT = Amps.of(40);

    public static final double KICKER_JAMMED_TIME = 0.2;


    public static final double KICKER_VELOCITY_TOLERANCE = 0.01; // Placholder Value

    public static final int KICKER_CURRENT_LIMIT = 80;



}
