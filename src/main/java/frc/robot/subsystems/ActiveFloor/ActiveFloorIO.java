package frc.robot.subsystems.ActiveFloor;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ActiveFloorIO {

    @AutoLog
    public static class ActiveFloorIOInputs {
        public Voltage activeFloorVoltage = Volts.of(0.0);
    }

    default void updateInputs(ActiveFloorIOInputs inputs) {}

    default void setVoltage(Voltage voltage) {}
    
     default void stop() {}
    
     default void setBreakMode(boolean enabled) {}
}
