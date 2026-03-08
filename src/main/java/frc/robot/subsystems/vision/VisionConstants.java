package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final String[] camNames = {
    "Back_Camera", "Rback_Camera", "Rfront_Camera"
  };
  public static final int numCameras = camNames.length;

  // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.
  //     public static final Transform3d[] camsRobotToCam = {
  //       new Transform3d(
  //           new Translation3d(
  //               Units.inchesToMeters(15) - 0.0178,
  //               Units.inchesToMeters(0.25),
  //               Units.inchesToMeters(7.5)),
  //           new Rotation3d(0, Units.degreesToRadians(15), 0)),
  //       new Transform3d(
  //           new Translation3d(
  //               Units.inchesToMeters(6.4375), Units.inchesToMeters(10.5),
  //   Units.inchesToMeters(35)),
  //           new Rotation3d(
  //               0, Units.degreesToRadians(25.5), Units.degreesToRadians(0))),
  //       new Transform3d(
  //           new Translation3d(
  //               Units.inchesToMeters(6.4375), Units.inchesToMeters(-10),
  // Units.inchesToMeters(35)),
  //           new Rotation3d(
  //               0, Units.degreesToRadians(25.5), Units.degreesToRadians(0))),
  //       new Transform3d(
  //           new Translation3d(
  //               Units.inchesToMeters(1.4375), Units.inchesToMeters(0.25),
  //   Units.inchesToMeters(31.75)),
  //           new Rotation3d(0, Units.degreesToRadians(16.7), 0))
  //     };

  public static final Transform3d[] camsRobotToCam = {
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(1.5), Units.inchesToMeters(-8.5), Units.inchesToMeters(20)),
        new Rotation3d(0, Units.degreesToRadians(0), 180)),
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.625), Units.inchesToMeters(-12.25), Units.inchesToMeters(8.5)),
        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-128.75))),
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(6.4375), Units.inchesToMeters(-10), Units.inchesToMeters(8.5)),
        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(28.75))),
  };

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final double AMBIGUITY_THRESHOLD = 0.2;
  public static final double MAX_DISTANCE = 4; // meters

  // The standard deviations of our vision estimated poses, which affect
  // correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 20);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 20);

  public static Transform3d getSimVersion(Transform3d real) {
    return new Transform3d(real.getTranslation(), new Rotation3d(0, 0, real.getRotation().getZ()));
  }
}
