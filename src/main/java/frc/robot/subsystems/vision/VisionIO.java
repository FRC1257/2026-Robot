package frc.robot.subsystems.vision;

import edu.wpi.first.cscore.VideoListener;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/*
 * automatically generation VisionIOInputsAutoLogged
 * allows every single field to be recorded for later
 * Connected = boolean to literally say if it's connected or not
 * Array of PoseObservations = contains all the pose observations from the limelight (timestamp, pose, tag count, ambiguity, avg tag distance, and type)
 * done this way cause we'll be taking into data from megatag 1 and 2; make sure we're getting both
 * number of tag ids we can see
 */

public interface VisionIO {
 @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public TargetObservation latestTarget = new TargetObservation(false, 0, 0);
    public int[] tagIds = new int[0]; 
  }


/*
 * timestamp when we saw the time
 * Pose2d (x, y, rotation)
 * tagCount = number of tags
 * ambiguity = how much noise is in the data (0 is perfect, higher is worse)
 * avgTagDist = average distance to the tags we can see (lower is better)
 * type = either MG 1 or 2
 */
public static record PoseObservation(
    double timestamp, 
    Pose2d pose, 
    int tagCount, 
    double ambiguity, 
    double avgTagDist, 
    PoseObservationType type) {}


  
  //for autoaim
  public static record TargetObservation(boolean hasTarget, double tx, double ty) {}

  public enum PoseObservationType { MEGATAG_1, MEGATAG_2 }
  

  //pass in rotation of the robot for megatag 2
  public default void updateInputs(VisionIOInputs inputs, Rotation2d gyroHeading) {}
}