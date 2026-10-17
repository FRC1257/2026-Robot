package frc.robot.util.shooter;

/** Pure feed policy: elapsed waiting time never substitutes for readiness. */
public final class ShotReadiness {
  public enum State { SPINUP, FEED, RECOVER, FAULT }
  private final double dwellSeconds;
  private double readySince = Double.NaN;
  private State state = State.SPINUP;
  public ShotReadiness(double dwellSeconds) {
    if (!Double.isFinite(dwellSeconds) || dwellSeconds < 0) throw new IllegalArgumentException();
    this.dwellSeconds = dwellSeconds;
  }
  public void reset() { readySince=Double.NaN; state=State.SPINUP; }
  public boolean update(double now, boolean valid, boolean startReady, boolean recoveryReady) {
    if (!Double.isFinite(now) || !valid) { readySince=Double.NaN; state=State.FAULT; return false; }
    if (state==State.FEED && recoveryReady) return true;
    if (state==State.FEED) state=State.RECOVER;
    if (!startReady) { readySince=Double.NaN; if(state==State.FAULT) state=State.SPINUP; return false; }
    if (!Double.isFinite(readySince) || now<readySince) readySince=now;
    if (now-readySince>=dwellSeconds) { state=State.FEED; return true; }
    return false;
  }
  public State state() { return state; }
}
