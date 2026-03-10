package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class NautilusMechanism3d {
    private static NautilusMechanism3d measured;

    public static NautilusMechanism3d getMeasured() {
        if (measured == null) {
            measured = new NautilusMechanism3d();
        }
        return measured;
    }
    
    private Angle hoodAngle = Radians.of(0.0);
    private Angle intakeAngle = Radians.of(-1.6);
    
    public void log(String key) {
        var hoodPose = new Pose3d(new Translation3d(0,0,0), new Rotation3d(Radians.of(0.0), hoodAngle, Radians.of(0.0)));
        var intakePose = new Pose3d(new Translation3d(0,0,0), new Rotation3d(Radians.of(0.0), intakeAngle, Radians.of(0.0)));
        Logger.recordOutput(key + "/Components", hoodPose, intakePose);
    }

    public void setHoodAngle(Angle angle) {
        hoodAngle = angle;
    }

    public void setIntakeAngle(Angle angle) {
        intakeAngle = angle;
    }

    public Angle getHoodAngle() {
        return hoodAngle;
    }

    public Angle getIntakeAngle() {
        return intakeAngle;
    }
}
