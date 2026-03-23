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
    "limelight-hopper", "limelight-swerve"
  };


  //ADVANTAGEKIT LL ADDITIONS

  public static String camera0Name = "limelight-hopper";
  public static String camera1Name = "limelight-swerve";
    // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  public static AprilTagFieldLayout aprilTagLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
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
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180))),
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.75), Units.inchesToMeters(-12.75), Units.inchesToMeters(8.5)),
        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-128.75))),
    //new Transform3d(
      //  new Translation3d(
         //   Units.inchesToMeters(12.9375), Units.inchesToMeters(-11.125), Units.inchesToMeters(8.5)),
       // new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(28.75))),
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