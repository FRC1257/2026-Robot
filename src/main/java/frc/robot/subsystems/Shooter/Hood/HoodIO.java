package frc.robot.subsystems.Shooter.Hood;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        Angle hoodAngle; 
        AngularVelocity hoodVelocity;
        Voltage hoodVolts; 
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void runVoltage(Voltage volts) {}

    
    
}
