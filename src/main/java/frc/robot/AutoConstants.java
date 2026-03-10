package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoConstants {
    public static final Pose2d LEFT_TRENCH_START_POSITION = new Pose2d(new Translation2d(3.509, 7.457), new Rotation2d(0.0));
    public static final Pose2d LEFT_TRENCH_END_POSITION = new Pose2d(new Translation2d(7.900, 5.443), new Rotation2d(Math.PI/2));

    public static final Pose2d RIGHT_TRENCH_START_POSITION = new Pose2d(new Translation2d(3.509, 0.612), new Rotation2d(0.0));

    public static final Pose2d DEPOT_START_POSITION = new Pose2d(new Translation2d(3.513, 5.965), new Rotation2d(0.0));

}
