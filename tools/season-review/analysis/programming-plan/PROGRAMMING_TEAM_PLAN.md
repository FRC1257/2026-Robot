# 1257 programming plan — same hardware

Prepared October 17, 2026. Plan against local HEAD `f1a5fe50d65188cd047efaab31455f381f28a509` plus the existing vision-telemetry working changes. No performance-control changes were applied or deployed. The four accompanying patches are review proposals: `git apply --check` passed individually; they have not been compiled or tested on a robot. Existing vision edits are preserved. Historical dirty builds prevent assigning every source defect to a match.

## Priority and expected difference

| Order | Deliverable / suggested owner | Why it matters on this robot | Effort estimate / acceptance gate |
|---|---|---|---|
| P0 | Preflight device validity + exact deployment manifest / IO owner | Avoid starting another match with Q7/Q11-style missing drive feedback or Q42-style invalid device data. Potentially protects much more match time than speed tuning. Software cannot repair Q11's wiring/power cause. | 1–2 programming sessions, then supervised device fault-injection. All required device failures identifiable before enable. |
| P1 | Finite autos and cancellation / command owner | Highest-confidence small correction: some retained auto paths cannot reach the next stage. | 1–2 sessions plus field runs. Every chooser option completes or reaches a declared fallback; interrupted outputs stop. |
| P2 | Shared shot coordinator / shooter + controls owners | Largest plausible sustained scoring-throughput gain once electrical health is sound. Q34 had 29.040/42.004 s feeding with flywheel readiness false; not a miss count. | 3–5 sessions plus calibration. First-shot latency and made balls per opportunity improve without a lower measured success rate. |
| P3 | Combined-demand limiting / drive + shooter owners | Q48 spent 67.837/140.046 teleop seconds below 9 V. Limits should protect steering and useful shots rather than raise all motor demand. | 2–3 sessions plus paired loaded tests. Better voltage margin with equal or better measured cycle productivity. |
| P4 | Heading and vision correctness / localization owner | Small corner-case fixes immediately; freshness/uncertainty work after command correctness. Protects alignment and pose trust. | Heading/bounds patch first session; freshness 2–3 sessions. Wrap, stale, duplicate and invalid observations tested. |
| P5 | Calibrated fuel estimator / telemetry owner | Gives future tuning a useful throughput metric without claiming motor-on time equals balls. Diagnostic only until calibrated. | 1–2 sessions for logging plus labeled trials. Held-out count error and confidence reported. |

Efforts are planning estimates for a student team, not measured delivery times. P0/P1 and the two small P4 fixes belong in the first working session; P2 follows before path-speed optimization. Do not claim a quantified points gain from the archive.

## 1. Make autonomous stages finite

File: `src/main/java/frc/robot/util/autonomous/AutoChooser.java`, RIGHT_TRENCH return path around line 73.

```diff
- .andThen(drive.followPathFileCommand("RNZ_RS").alongWith(flywheel.runIdle()))
+ .andThen(drive.followPathFileCommand("RNZ_RS").deadlineWith(flywheel.runIdle()))
```

Exact review patch: `01-auto-deadline.patch`. `alongWith` waits for every command; `runIdle()` runs continuously. `deadlineWith` lets path completion interrupt idle spin so scoring starts. This follows [WPILib command-composition semantics](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html). It applies to RIGHT_TRENCH; do not assume it explains the distinct RIGHT_TRENCH_SWEEP match behavior.

DEPOT around line 80 also has an unbounded `hopperPivot.runIntakeAngle()` before DEPOT_END. Replace it with a bounded move and a success branch that owns the entire remaining continuation. Pattern using existing APIs:

```java
hopperPivot.runIntakeAngle()
    .until(hopperPivot.atGoal())
    .withTimeout(pivotMoveTimeoutSeconds)
    .andThen(Commands.either(
        drive.followPathFileCommand("DEPOT_END")
            .deadlineWith(hopperIntake.runIntake())
            .andThen(targetedScore()),
        hopperPivot.setPivotVoltage(() -> edu.wpi.first.units.Units.Volts.of(0)),
        () -> hopperPivot.atGoal().getAsBoolean()));
```

**Integration specification, not a drop-in patch:** introduce and calibrate `pivotMoveTimeoutSeconds`; the failure branch uses the existing `setPivotVoltage` API to hold zero command until interrupted. Verify whether zero voltage or a position hold is physically appropriate before using it. Do not place a failed stage's `andThen(nextPath)` outside the branch, which would let failure continue anyway. Use fresh command instances. Apply the same bounded-success/failure structure to every initial pivot/alignment wait. Log `Auto/Stage`, `StageStartedAt`, `StageExitReason` (success/timeout/interrupted/invalid-feedback), chosen path and final outcome.

Intake cancellation deserves a separate fix: `HopperIntake.runVoltage` currently uses `run`, which has no explicit end action. Patch `04-intake-stop-on-cancel.patch` changes it to `runEnd(..., io::stop)`. Test interaction with defaults; current capture is a source observation, not proof an intake historically ran unexpectedly. Audit ActiveFloor and HopperPivot end behavior too.

**Scheduler tests:** fake I/O, simulated clock, actual chooser compositions. Path completes → idle command ends → scoring starts; pivot never at goal → timeout → fallback → next path never starts; cancellation and disable stop every applicable output within a scheduler cycle. Exercise all five options and both alliances. Test a companion command ending early so collection cannot inadvertently shorten a path. Then run one supervised unloaded test and at least five repeatable loaded field trials for the chosen competition auto. Simulation does not certify collision clearance or physical shot success.

## 2. Replace one-time feed permission with a shared shot coordinator

Files: `RobotContainer.java` right/left bumper bindings around lines 254–285; `AutoChooser.targetedScore()`; new `commands/ShotCoordinator.java` and pure `util/shooter/ShotReadiness.java`.

Remove both `.waitUntil(ready).withTimeout(0.5).andThen(feed)` chains. Timeout is a second way to end the wait; it does not establish readiness. Auto's initial readiness check likewise does not continually protect feeding. Use the same policy in auto and driver operation, with different aim modes, not three drifting implementations.

Implement an explicit state transition each 20 ms cycle:

```java
// Proposed pure policy, with all thresholds passed in a calibrated config.
if (!requested) state = IDLE;
else if (!feedbackValid || !targetValid) state = FAULT;
else if (overrideRequested) state = OVERRIDE;
else if (state == FEED && !insideRecoveryBand) state = RECOVER;
else if (readyDwellSatisfied) state = FEED;
else if (spinupTimedOut) state = FAULT;
else if (state != RECOVER) state = SPINUP;
boolean feedAllowed = state == FEED || state == OVERRIDE;
```

This is a specification fragment; reset dwell and timeout at the appropriate request/state transitions, and define the override policy deliberately. Require fresh device feedback even for an ordinary driver override; never let an override bypass framework disable or electrical protection. On release/cancel, stop the kicker/floor and set a deliberate intake/pivot hold behavior.

Add `Kicker.runGatedIntake(BooleanSupplier allowFeed)` using the same intake setpoint and `runEnd` shutdown as the existing method. Evaluate `allowFeed` every execute cycle; when false call `io.stop()`. Add analogous gated floor behavior and explicit agitation/collection policy. Let one command own each subsystem; don't schedule separate commands for the same mechanism in a parallel group. Calculate one shot solution (distance, hood goal, flywheel goal, heading) per cycle and share it across subsystems. Log requested and applied solutions separately.

Keep the existing ±2.5 rad/s readiness band as an initial comparison setting, not a proven optimal threshold. Calibrate start and recovery bands with hysteresis and dwell using counted shots; avoid stopping for harmless ripple. Include hood error, wrapped heading error, pose validity and chassis-motion criteria. Replace the one-time `Commands.either` drive/lock choice in the right-bumper binding with an execute-time mode decision, so moving the stick after alignment can reliably leave the lock state.

**Tests:** never ready for >0.5 s → no feed; ready for shorter than dwell → no feed; valid stable ready → feed; speed falls outside recovery band → stop/reduce feed → recover → resume; invalid sensor, target jump, button release, timeout and disable each take the expected state. Test override logs, subsystem requirements and ±π heading crossing. Replay old logs in shadow mode only: old inputs cannot predict the physical closed-loop result of a different controller.

**Physical comparison:** count made/attempted releases at several fixed distances, matched starting inventory and battery state, alternating old/new policy order. Measure first-valid-shot latency, total made balls per opportunity and recovery time. At least ten repeated opportunities per condition is an initial sampling plan, not a statistical guarantee. Agree on success criteria before viewing results.

## 3. Preflight health and deployment identity

Files: `subsystems/drive/ModuleIO.java`, `ModuleIOSparkMax.java`, `Module.java`, mechanism IO classes, `Robot.java`; new `util/health/RobotHealth.java`.

Add separate per-drive/per-steer `connected`, `lastSuccessfulReadTimestamp`, last read/config/setpoint error, controller bus voltage and supported fault/reset fields. Only refresh success time after successful device reads; a repeated valid zero is different from stale/failed input. Keep old encoder fields for replay compatibility. Use reported controller error codes, not current=0 as a connection test.

Compute required-device health and log each inhibit reason. Warn for commanded drive motion plus moving peers and invalid/persistently zero module feedback, with a deliberate debounce and near-zero exclusions. Do not reconstruct missing feedback from other modules and silently insert it into odometry. Provide a supervised preflight result and a logged driver acknowledgment; do not automatically actuate on enable. Review any auto-inhibit/degraded-drive policy with drivers before adoption.

At build/deploy, record exact source archive/hash, dirty diff hash, path hashes, camera transforms, shot-table/config revision, library versions and controller-ID map. Do not log credentials. Attach a battery identifier manually if no sensor exists.

**Tests:** disconnected drive with steering alive, zero velocity while stationary, wheel commanded zero during a turn, CAN read error, duplicate/stale packets and recovery. Verify all warnings distinguish missing from zero and clear only after healthy evidence. On a powered robot use vendor diagnostics and mentor-supervised procedures; do not disconnect live high-current wiring to generate failures.

## 4. Manage combined demand

Files: `Drive.runVelocity` and drive setpoint generation; `Flywheel.runVelocity`/goal update; new `util/power/PowerBudget.java`.

Ramp chassis acceleration and flywheel targets using measured elapsed time. Start with logged-only demand decisions, then enable a calibrated policy that smoothly reduces translational acceleration and spin-up ramp under low-voltage conditions while preserving steering/control authority. Restore demand with hysteresis. Do not set 9 V as an automatic threshold merely because the retrospective used it for exploration. Do not alter brownout protection or breaker ratings.

Use RIO voltage plus validated battery-side current. Do not close the loop around the suspect PDH channels until decoder validity is established. Replace the present EnergyLogger's sum-of-motor-current approximation with trusted battery-side power integration and measured dt, or label it explicitly as an estimate.

**Tests:** simultaneous drive acceleration + shooter spin-up + intake, low battery, noisy voltage, abrupt dt changes and recovery. Verify finite outputs and rate limits, no oscillation around thresholds, and unchanged stop/disable behavior. Paired physical trials must improve made balls/cycle or availability, not merely flatten a voltage plot by making everything slow.

## 5. Heading and vision corrections

Exact `02-heading-wrap.patch`, `Drive.isHubAligned()` around line 649:

```diff
- Math.abs(rotationSupplier.getRadians()-currentRotation.getRadians())
+ Math.abs(rotationSupplier.minus(currentRotation).getRadians())
```

Test target −179°, robot +179° → approximately 2° error; reversed signs; 0°/90°; exact 5° boundary. Keep the current tolerance until shot trials justify changing it.

Exact `03-vision-bounds.patch` moves array-size checks before the first `stdDeviations.get(i)` and includes `rotationEstimates`. Test shorter deviation/timestamp/rotation arrays and empty input. This patch does not yet solve duplicate observations.

In `VisionIOPhoton.getLatestResult` and `Drive.periodic`, add a stable camera identity, per-camera last-consumed timestamp and observation validity. Do not assume an array index remains a camera ID if estimates are compacted. Reject duplicates/out-of-order frames, nonfinite timestamps/pose/uncertainty, and frames older than a calibrated age limit. Mark an observation consumed once considered, including rejected observations, and log the rejection reason; make reset/reconnect policy explicit. Prevent old cached frames from masquerading as new arrivals. Tests: no new frames, duplicate timestamp, out-of-order frame, camera restart, stale frame, implausible pose and valid recovery. Compare pose against measured field references, not camera connection flags.

## Handoff and release process

Keep one pull request per behavior change. First run Java scheduler/unit tests on the team's pinned WPILib environment, then simulation, then supervised physical checks. Record tested revision/configuration and retain before/after logs. The proposed patches were checked for applicability only; this plan does not claim those future tests passed. The offline ball-probe tests are separate and cannot validate robot control code.

Use the acceptance gates above instead of “it compiled” or “we won.” Electrically healthy match time comes first, correct auto transitions second, measured shooting throughput third, speed optimization last.
