package frc.robot.subsystems.Shooter.Hood;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        Angle positionRads; 
        AngularVelocity velocityRadsPerSec;
        Voltage appliedVolts; 
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setAngle(Angle angle) {}

    public default void zero() {}

    
}
