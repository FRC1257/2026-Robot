package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

/** Diagnostic logging only; does not change which measurements enter the pose estimator. */
public final class VisionTelemetry {
  /** Display freshness limit, not an estimator acceptance threshold. */
  static final double DISPLAY_MAX_AGE_SECONDS = 0.25;

  private VisionTelemetry() {}

  record FrameBatch(double[] timestamps, long[] targetCounts, long[] targetIds,
      long[] frameIndices, double[] yaw, double[] pitch, double[] ambiguity,
      Transform3d[] cameraToTargets) {}

  static FrameBatch flatten(List<PhotonPipelineResult> frames) {
    int count = frames.stream().mapToInt(f -> f.getTargets().size()).sum();
    double[] timestamps = new double[frames.size()];
    long[] targetCounts = new long[frames.size()];
    long[] ids = new long[count];
    long[] frameIndices = new long[count];
    double[] yaw = new double[count];
    double[] pitch = new double[count];
    double[] ambiguity = new double[count];
    Transform3d[] transforms = new Transform3d[count];
    int targetIndex = 0;
    for (int frameIndex = 0; frameIndex < frames.size(); frameIndex++) {
      var frame = frames.get(frameIndex);
      timestamps[frameIndex] = frame.getTimestampSeconds();
      targetCounts[frameIndex] = frame.getTargets().size();
      for (var target : frame.getTargets()) {
        ids[targetIndex] = target.getFiducialId();
        frameIndices[targetIndex] = frameIndex;
        yaw[targetIndex] = target.getYaw();
        pitch[targetIndex] = target.getPitch();
        ambiguity[targetIndex] = target.getPoseAmbiguity();
        transforms[targetIndex] = target.getBestCameraToTarget();
        targetIndex++;
      }
    }
    return new FrameBatch(timestamps, targetCounts, ids, frameIndices, yaw, pitch, ambiguity, transforms);
  }

  /** Preserve all frames returned by PhotonVision, including empty/high-ambiguity frames. */
  public static void recordUnreadFrames(String name, List<PhotonPipelineResult> frames) {
    var batch = flatten(frames);
    String key = "Vision/CameraObservations/" + name;
    Logger.recordOutput(key + "/FrameTimestamps", batch.timestamps());
    Logger.recordOutput(key + "/FrameTargetCounts", batch.targetCounts());
    Logger.recordOutput(key + "/TargetIds", batch.targetIds());
    Logger.recordOutput(key + "/TargetFrameIndices", batch.frameIndices());
    Logger.recordOutput(key + "/YawDegrees", batch.yaw());
    Logger.recordOutput(key + "/PitchDegrees", batch.pitch());
    Logger.recordOutput(key + "/PoseAmbiguities", batch.ambiguity());
    Logger.recordOutput(key + "/BestCameraToTargets", batch.cameraToTargets());
  }

  record TrackedTags(long[] ids, Pose3d[] poses) {}

  static TrackedTags trackedTags(PhotonPipelineResult[] frames, boolean[] connected, double now) {
    List<Long> ids = new ArrayList<>();
    List<Pose3d> poses = new ArrayList<>();
    for (int i = 0; i < Math.min(frames.length, connected.length); i++) {
      double age = now - frames[i].getTimestampSeconds();
      if (!connected[i] || age < 0 || age > DISPLAY_MAX_AGE_SECONDS) continue;
      for (var target : frames[i].getTargets()) {
        var pose = VisionConstants.kTagLayout.getTagPose(target.getFiducialId());
        if (pose.isPresent() && !ids.contains((long) target.getFiducialId())) {
          ids.add((long) target.getFiducialId());
          poses.add(pose.get());
        }
      }
    }
    return new TrackedTags(ids.stream().mapToLong(Long::longValue).toArray(), poses.toArray(Pose3d[]::new));
  }

  /** Latest fresh detections, before estimator quality filtering, at known field tag locations. */
  public static void recordTrackedTags(PhotonPipelineResult[] frames, boolean[] connected, double now) {
    var tags = trackedTags(frames, connected, now);
    Logger.recordOutput("Vision/TrackedTags/Ids", tags.ids());
    Logger.recordOutput("Vision/TrackedTags/FieldPoses", tags.poses());
    for (int i = 0; i < VisionConstants.numCameras; i++) {
      String key = "Vision/CameraObservations/" + VisionConstants.camNames[i];
      boolean active = i < frames.length;
      double timestamp = active ? frames[i].getTimestampSeconds() : -1;
      double age = active ? now - timestamp : -1;
      boolean fresh = active && connected[i] && age >= 0 && age <= DISPLAY_MAX_AGE_SECONDS;
      Logger.recordOutput(key + "/Active", active);
      Logger.recordOutput(key + "/Connected", connected[i]);
      Logger.recordOutput(key + "/LatestFrameTimestamp", timestamp);
      Logger.recordOutput(key + "/LatestFrameAgeSeconds", age);
      Logger.recordOutput(key + "/Fresh", fresh);
      Logger.recordOutput(key + "/LatestTargetIds", fresh
          ? frames[i].getTargets().stream().mapToLong(t -> t.getFiducialId()).toArray() : new long[0]);
    }
  }
}
