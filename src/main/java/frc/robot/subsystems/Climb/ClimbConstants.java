package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {
    public static final int CLIMB_MOTOR_ID = 14;
    public static final Voltage RETRACT_VOLTAGE = Volts.of(0.0);
    public static final Voltage HOLD_VOLTAGE = Volts.of(0.0);
    public static final Voltage EXTEND_VOLTAGE = Volts.of(0.0);
    public static final Current CLIMB_CURRENT = Amps.of(40);
}
