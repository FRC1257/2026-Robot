package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class HoodConstants {

    public static final Angle HOOD_MIN_ANGLE = Radians.of(0);
    public static final Angle HOOD_MAX_ANGLE = Radians.of(0.432);

    

    public static final double HOOD_ANGLE_TOLERANCE = Degrees.of(0.5).in(Radians);

    public static final int HOOD_MOTOR_ID = 10;

    public static final double HOOD_KP = 0.0;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 0.0;
    public static final double HOOD_KS = 0.0;
    public static final double HOOD_KV = 0.0;
    public static final double HOOD_KG = 0.0;

    public static final TrapezoidProfile.Constraints HOOD_CONSTRAINTS = new TrapezoidProfile.Constraints(3.0, 6.0);
    public static final double HOMING_VELOCITY_THRESHOLD = 0.5; 
    public static final Voltage HOMING_VOLTAGE = Volts.of(-3.0);
    public static final boolean HOOD_INVERTED = true;
    public static final IdleMode HOOD_IDLE_MODE = IdleMode.kBrake;
    public static final int HOOD_SMART_CURRENT_LIMIT = 80;
    public static final double HOOD_POSITION_CONVERSION_FACTOR = 2*Math.PI*1/34.6;
    public static final double HOOD_VELOCITY_CONVERSION_FACTOR = HOOD_POSITION_CONVERSION_FACTOR/60;
    public static final double ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR = 2*Math.PI;
    public static final double ABSOLUTE_ENCODER_VELOCITY_CONVERSION_FACTOR = ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR/60;
    public static final boolean ABSOLUTE_ENCODER_INVERTED = false;
    
}
