package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ClimbIO {

    @AutoLog
    public static class ClimbIOInputs {
        Voltage appliedVolts = Volts.of(0.0);
        Current currentAmps = Amps.of(0.0);
        Temperature temperatureCelcius = Celsius.of(0.0);
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setVoltage(Voltage volts) {}
    
}
