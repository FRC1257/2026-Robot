package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionTelemetryTest {
  private PhotonTrackedTarget target(int id, double ambiguity) {
    var target = new PhotonTrackedTarget();
    target.fiducialId = id;
    target.poseAmbiguity = ambiguity;
    target.bestCameraToTarget = new Transform3d();
    return target;
  }

  private PhotonPipelineResult frame(long sequence, double time, PhotonTrackedTarget... targets) {
    return new PhotonPipelineResult(sequence, (long) (time * 1e6),
        (long) (time * 1e6), 0, List.of(targets));
  }

  @Test void retainsEveryUnreadFrameIncludingEmptyAndRejectedQuality() {
    var batch = VisionTelemetry.flatten(List.of(frame(1, 9.9, target(1, 0.9), target(2, 0.1)),
        frame(2, 9.95), frame(3, 10.0, target(3, 0.2))));
    assertArrayEquals(new long[] {2, 0, 1}, batch.targetCounts());
    assertArrayEquals(new long[] {1, 2, 3}, batch.targetIds());
    assertArrayEquals(new long[] {0, 0, 2}, batch.frameIndices());
    assertArrayEquals(new double[] {9.9, 9.95, 10.0}, batch.timestamps(), 1e-6);
    assertEquals(0.9, batch.ambiguity()[0]);
  }

  @Test void duplicateIdsAcrossCamerasProduceOneFieldMarker() {
    var tags = VisionTelemetry.trackedTags(new PhotonPipelineResult[] {
        frame(1, 9.9, target(1, 0.9)), frame(2, 9.95, target(1, 0.1), target(2, 0.1))},
        new boolean[] {true, true}, 10.0);
    assertArrayEquals(new long[] {1, 2}, tags.ids());
    assertEquals(2, tags.poses().length);
  }

  @Test void staleDisconnectedFutureAndUnknownTagsDoNotInventFieldMarkers() {
    var tags = VisionTelemetry.trackedTags(new PhotonPipelineResult[] {
        frame(1, 9.0, target(1, 0.1)), frame(2, 9.9, target(2, 0.1)),
        frame(3, 10.1, target(3, 0.1)), frame(4, 9.9, target(999, 0.1))},
        new boolean[] {true, false, true, true}, 10.0);
    assertEquals(0, tags.ids().length);
    var raw = VisionTelemetry.flatten(List.of(frame(4, 9.9, target(999, 0.1))));
    assertArrayEquals(new long[] {999}, raw.targetIds());
  }
}
