package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Map;

/** Per-camera observation consumption; camera indices are stable in VisionIO. */
public final class VisionFrameGate {
  private final Map<Integer, Double> consumed = new HashMap<>();
  private final double maxAge;
  public VisionFrameGate(double maxAge) {
    if (!Double.isFinite(maxAge) || maxAge<=0) throw new IllegalArgumentException();
    this.maxAge=maxAge;
  }
  public String check(int camera, double timestamp, double now) {
    if (!Double.isFinite(timestamp) || !Double.isFinite(now) || timestamp<=0) return "invalid-time";
    if (timestamp>now+0.05) return "future"; // Do not poison the camera watermark.
    if (timestamp<=consumed.getOrDefault(camera,Double.NEGATIVE_INFINITY)) return "duplicate-or-old";
    consumed.put(camera,timestamp);
    if (now-timestamp>maxAge) return "stale";
    return "accepted";
  }
}
