package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {

    }
    public default void updateInputs(KickerIOInputs inputs) {}
}
