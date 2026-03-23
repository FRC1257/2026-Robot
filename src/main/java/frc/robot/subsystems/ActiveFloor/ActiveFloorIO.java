package frc.robot.subsystems.ActiveFloor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ActiveFloorIO {

    @AutoLog
    public static class ActiveFloorIOInputs {
        public boolean activeFloorConnected = false;
        public Voltage activeFloorVoltage = Volts.of(0.0);
        public AngularVelocity activeFloorAngularVelocity = RadiansPerSecond.of(0.0);
        public Current activeFloorCurrent = Amps.of(0.0);
        public Temperature activeFloorTemperature = Celsius.of(0.0);
    }

    default void updateInputs(ActiveFloorIOInputs inputs) {}

    default void setVoltage(Voltage voltage) {}
    
    default void stop() {}
    
    default void setBreakMode(boolean enabled) {}
}
