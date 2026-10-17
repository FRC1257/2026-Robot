# Camera perspectives, AprilTags and robot pose

The saved AdvantageScope layout includes **7 Camera Back_Camera**, **8 Camera SwerveBack**, **9 Camera SwerveFront**, and **10 Tag IDs + Pose**. Command+Left / Command+Right switch tabs when later tabs are off-screen. Select a time on the timeline or start playback to give the camera a robot pose.

These are reconstructed views of the 2026 field from the robot's estimated pose and code-defined camera transforms. They are not recorded camera images. Lens distortion, exposure, real lighting and actual obstructions are not reconstructed.

| Camera / log index | Robot-relative position, meters (x forward, y left, z up) | Roll / pitch / yaw, degrees |
| --- | --- | --- |
| Back_Camera / 0 / cam1 | (0.038100, −0.215900, 0.508000) | (0, 0, 180) |
| SwerveBack / 1 / cam2 | (0.273050, −0.323850, 0.215900) | (0, −25, −128.75) |
| SwerveFront / 2 / cam3 | (0.328613, −0.282575, 0.215900) | (0, −25, 28.75) |

Source: `code/src/main/java/frc/robot/subsystems/vision/VisionConstants.java` at `f1a5fe50d65188cd047efaab31455f381f28a509`. The committed source revisions named by Tabernacle Q11 (`f0e5fc88…`) and Bensalem Q48 (`eaa6f817…`) contain the same active transforms. Both logs report uncommitted source changes, so this does not establish the exact deployed calibration. The historical files are retained in `../provenance/`.

The 480×320 image shape and approximately 60.45° horizontal FOV are derived from the **70° diagonal simulation calibration** in `VisionIOSim.java`; the physical cameras' intrinsics have not been supplied or verified. AdvantageScope approximates vertical FOV internally. Camera orientations use ordered Y-pitch then Z-yaw rotations, equivalent to the code's zero-roll `Rotation3d`.

The normal robot tab uses the supplied approximate robot model. The camera tabs use a separate camera rig with the same sourced transforms and no robot body: the illustrative model's solid hopper would otherwise hide the field from a camera inside it. This avoids presenting approximate body occlusion as a measured camera obstruction. Actual body geometry and camera clearance need CAD or physical inspection.

## Historical records

All 136 original log files were inspected for vision and pose data. The export retained **5,550,915 recorded changes** across these signals in `vision-records/*.jsonl.gz`. These are log changes, not unique video frames, detections or scored actions. `vision-export-index.json` records per-file counts, hashes and errors. `tag-coverage.csv` lists unique recorded IDs per camera index. Nine source files are empty, and three exports stop at a decode error after retaining their readable prefix.

- `/Vision/CameraTargets/0`, `/1`, `/2`: camera-indexed ID arrays already recorded by the team. In existing code these update only inside `hasEstimate(results)` and may remain unchanged when a later frame is rejected or no target is seen. A held ID is not proof of a current detection.
- `/RealOutputs/Vision/Targets3D`: known field positions of tags from quality-qualified camera results. Orange lines in the robot and camera tabs show these legacy targets; the source field can remain stale. They are not measured camera-to-tag poses and do not necessarily enumerate every detected tag.
- `/Vision/TimestampArray` and `/Vision/HasEstimate`: retained timing/status context for interpreting those legacy arrays.
- `/RealOutputs/Odometry/Robot`: the team's fused estimated robot pose, not ground truth. `/RealOutputs/Odometry/Odometry` is the wheel/gyro odometry comparison. `/Vision/PositionEstimates` contains camera estimates; zero poses are used as missing-estimate sentinels in the team code.

The export preserves raw FPGA record timestamps and exact source value ordering. Pose2d tuples are `[x_m, y_m, yaw_rad]`; Pose3d tuples are `[x_m, y_m, z_m, qw, qx, qy, qz]`. Record time can differ from camera frame capture time. Nonexistent records were not reconstructed.

## Logging added for future runs

Changes are local in `../code/` on branch `codex/2026-log-review`, with a portable patch saved in `vision-logging.patch`. They have not been deployed to a robot. Estimator acceptance and control behavior are unchanged.

`Vision/CameraObservations/<camera>/` records **every frame returned by `getAllUnreadResults()`**, before selecting the latest frame or applying estimator quality checks:

- `FrameTimestamps` and `FrameTargetCounts` preserve empty frames and frame boundaries.
- `TargetIds`, `TargetFrameIndices`, `YawDegrees`, `PitchDegrees`, `PoseAmbiguities` and `BestCameraToTargets` preserve all targets in those received frames, including high-ambiguity and unknown IDs.
- `Active`, `Connected`, `LatestFrameTimestamp`, `LatestFrameAgeSeconds`, `Fresh` and `LatestTargetIds` distinguish disconnected, disabled and stale camera data. Side cameras intentionally disabled by the existing KillSideCams setting are not read; frames dropped upstream by PhotonVision cannot be recovered.

`Vision/TrackedTags/Ids` and `Vision/TrackedTags/FieldPoses` provide matched arrays for AdvantageScope AprilTag markers. They include known IDs observed by connected, active cameras within 0.25 seconds, without applying the estimator's ambiguity rejection. Duplicate IDs seen by multiple cameras get one field marker. Unknown IDs remain in the raw observation arrays but cannot get an invented field pose. Empty/stale snapshots clear the marker arrays. The 0.25-second limit is a display rule, not a measurement-quality threshold.

`Odometry/EstimatedPose` explicitly records the final fused pose after the estimator update. `Vision/RobotToCameras` records the configured extrinsics, and `Vision/CameraPoses` records the camera field poses computed from that estimate. These make future configuration and pose provenance inspectable in the log.

The new fields appear under `/RealOutputs/` in normal robot and simulator logs (verified in the smoke-test recording). AdvantageKit replay recomputation can use `/SimOutputs/`; the saved season layout targets the original `/RealOutputs/` signals.

## Validation

The modified project builds and its three JUnit tests pass. Tests cover multiple unread frames (including empty/high-ambiguity frames), duplicate IDs across cameras, and stale/disconnected/future/unknown-ID handling. Simulation startup was exercised and the new logging keys inspected in its recording; this is not hardware timing or camera calibration validation. Startup loop overruns remain visible in the smoke-test log.

The original generated GLBs had no vertex normals. AdvantageScope's mesh optimizer discards geometry without normals, which explained the missing robot. `configure_model.py` derives normals and converts each uniform mesh color into a material, preserving positions and triangle indices; tests verify unchanged geometry buffers and unit normals. Originals remain untouched in `models/`. Repairs and configuration source receipts are in `model-configuration.json`.
