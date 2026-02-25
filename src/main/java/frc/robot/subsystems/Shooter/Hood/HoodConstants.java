package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {

    public static final Angle HOOD_MIN_ANGLE = Radians.of(Units.degreesToRadians(0.0));
    public static final Angle HOOD_MAX_ANGLE = Radians.of(Units.degreesToRadians(45.0));

    

    public static final double HOOD_ANGLE_TOLERANCE = Degrees.of(1.0).in(Radians);

    public static final int HOOD_MOTOR_ID = 0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final TrapezoidProfile.Constraints HOOD_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 6.0);
    public static final double HOMING_VELOCITY_THRESHOLD = 0.5; 
    public static final Voltage HOMING_VOLTAGE = Volts.of(-3.0);
    
}
