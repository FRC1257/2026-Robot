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
    
    private Rotation2d hoodAngle = Rotation2d.kZero;
    private Rotation2d intakeAngle = Rotation2d.kZero;
    
    public void log(String key) {
        var hoodPose = new Pose3d(new Translation3d(0,0,0), new Rotation3d(0, hoodAngle.getRadians(), 0));
        var intakePose = new Pose3d(new Translation3d(-0.193,0, 0.205), new Rotation3d(0, -intakeAngle.getRadians(), 0));
        Logger.recordOutput(key + "/Components",intakePose, hoodPose);
    }

    public void setHoodAngle(Rotation2d angle) {
        hoodAngle = angle;
    }

    public void setIntakeAngle(Rotation2d angle) {
        intakeAngle = angle;
    }

    public Rotation2d getHoodAngle() {
        return hoodAngle;
    }

    public Rotation2d getIntakeAngle() {
        return intakeAngle;
    }
}
