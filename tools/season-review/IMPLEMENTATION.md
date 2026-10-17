# Programming implementation and release checks

Base: FRC1257/2026-Robot `f1a5fe50d65188cd047efaab31455f381f28a509`. Branch: `codex-analysis`. This is a review and test branch; no roboRIO deployment was performed.

| Priority | Change | Expected hardware benefit | Required robot acceptance |
|---|---|---|---|
| P0 | AutoStage bounds pivot stages to 2 seconds and selects an explicit zero-voltage fallback on failure. Return paths end continuous idle-spin companions using deadline semantics. Intake and floor stop on cancellation. | Avoid a stuck auto sequence or motors remaining commanded after cancellation. No claimed measured score gain. | Exercise every chooser option; force pivot timeout and path cancellation; verify safe pivot behavior under gravity and confirm scoring begins only after the intended stage. |
| P0 | Shared ShotFeed evaluates hood, alignment, flywheel readiness and reported feedback validity continuously; 100 ms readiness dwell and 5 rad/s recovery band. | Reduce feeding through spin-up or loss of aim; throughput effect must be measured. | Counted shot trials, battery sag, moving aim, disconnect and release tests. Tune dwell/bands against recovery and accuracy, not just speed. |
| P0 | Right-bumper targeting remains responsive to driver translation after alignment. | Avoid the old one-time branch decision leaving the driver in an inappropriate state. | Driver practice with right bumper held; verify alignment loss stops feeding. This mode no longer automatically X-locks. D-pad up remains explicit lock. |
| P1 | Per-camera duplicate, out-of-order, stale, future and invalid vision-frame rejection; wrapped heading difference. | Avoid repeatedly applying an old estimate and heading discontinuities near ±180°. | Camera dropout/reconnect, moving robot, time synchronization, noisy poses; inspect acceptance reasons. Maximum age 0.5 seconds and future tolerance 0.05 seconds are initial values. |
| P1 | Swerve drive/turn read-valid flags, error strings, device IDs, bus voltage and last successful read times; REAL alerts. | Make failed reads visible before a match and during review. | Disconnect/reconnect each controller on stands; verify module identification and alerts. Read status is not proof of fresh CAN packets or a physical wiring diagnosis. No automatic drive inhibit added. |
| P1 | Embedded SHA-256 manifest of src/main, vendordeps and build.gradle logged with existing Git metadata. | Reconstruct which configuration/source content produced a deployment, including dirty changes. | Open a new robot log and verify DeploymentManifest; retain built JAR/source archive. Hashes identify bytes, not calibrated physical configuration. |
| P2 | Logged candidate fuel events and video-linked labeling workflow. | Find likely collection/shooting intervals on existing hardware. | Counted calibration and held-out video validation before reporting ball counts. |

The AdvantageScope layouts, illustrative camera-rig GLB and camera/tag telemetry are included. The model is visualization geometry, not fabrication CAD. The old programming plan remains a historical proposal; this file describes what was actually implemented.

## Verification scope

Java unit/HAL tests cover bounded stage transitions, deadline cancellation, readiness dwell/recovery, feed-command output stopping, vision frame timing and tag-marker filtering. A Gradle build packages the manifest. Python tests cover log helpers, video server ranges/access boundaries, transient detector behavior and missing/reversed input. Browser checks cover match/kind filtering, event navigation, seeking, uncertain labeling and label export initiation.

These are not full physical autonomous runs or measured performance gains. Existing physics simulation is incomplete (notably KickerIOSim is empty and simulation connected flags are not populated); the new feed gate correctly remains unavailable when feedback is unknown. Command integration tests use explicit healthy fake IO/readiness. Do not infer calibrated simulator performance from test success.

## Deferred deliberately

Automatic power/load shedding, new trajectory speed limits, retuned flywheel gains, automatic jam reversal, a new autonomous route and calibrated ball counting are not deployed by this branch. Available evidence does not establish safe thresholds or physical dynamics for those changes. Hardware inspection remains necessary for Q11 communication loss and the front-right module symptoms. Diagnostics cannot repair a mechanical or wiring defect.

Release in small steps: first bench-check telemetry and cancellation, then drive/vision checks, then counted stationary shooting, then moving shooting and full autonomous practice. Retain before/after logs and scored-ball counts. Review each separate commit so behavior changes can be reverted individually.
