//needs a set voltage and run voltage, in java the stop would set voltage to 0,
package frc.robot.subsystems.Climb;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;


public interface ClimbIO{
   @AutoLog

public static class ClimbIOInputs {

    public AngularVelocity velocityRadsPerSec; 
    public Voltage appliedVoltage;
    public double[] tempCelcius;
    public double[] currentAmps;

}

    
public default void updateInputs(ClimbIOInputs inputs) {}

public default void setVoltage(Voltage voltage) {}

//public default void setBrake(boolean brake) {}

public default void stop() {}




}

