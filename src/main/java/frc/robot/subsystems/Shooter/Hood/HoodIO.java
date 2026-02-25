package frc.robot.subsystems.Shooter.Hood;


import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        Angle hoodAngle = Radians.of(0.0); 
        AngularVelocity hoodVelocity = RadiansPerSecond.of(0.0);
        Voltage hoodVolts = Volts.of(0.0); 
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void runVoltage(Voltage volts) {}


    
}
