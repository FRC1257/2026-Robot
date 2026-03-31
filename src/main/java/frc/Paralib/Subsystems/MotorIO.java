package frc.Paralib.Subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {

    @AutoLog
    public static class MotorInputs{
        public Angle positionRadians;
        public Distance positionMeters;
        public AngularVelocity velocityRadsPerSec;
        public Voltage voltageApplied;
        public Current currentAmps;
        public Temperature temperatureCelsius;
    }

    public void updateInputs(MotorInputs inputs);
    
    public void setVoltage(Voltage voltage);

    public void setVelocity(AngularVelocity velocity);

    public void setAngle(Angle position);

    public void setPosition(Distance position);

}  
