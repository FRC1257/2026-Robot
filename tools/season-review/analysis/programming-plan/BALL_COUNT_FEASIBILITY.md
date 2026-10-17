# Can we count fuel from the existing motor logs?

October 17, 2026. **Not yet with a defensible numerical accuracy.** An exploratory detector was run on all 13 enabled competition recordings in the supplied archive. We can measure motor activity and identify candidate disturbances. We cannot currently turn those disturbances into a calibrated count of acquired or launched balls, and neither measure establishes successful scores.

## What was actually tried

Read original WPILOG flywheel speed and goal, kicker voltage, intake voltage/current/speed, and RIO voltage. Use a 20 ms held-value grid for an exploratory event detector, retaining original source hashes. A candidate requires flywheel speed above 100 rad/s, kicker magnitude above 1 V, goal variation no more than 2 rad/s in the surrounding 200 ms, a 5/10/20 rad/s speed drop from the preceding 100 ms peak, at least half that recovery within 400 ms, and 160 ms between candidates. A stricter variant also requires battery voltage at least 9 V and no more than 0.5 V variation in that window. These settings are arbitrary exploratory thresholds, not calibration and not the robot's brownout threshold.

At high feed rates a continuous depression can merge multiple balls; a long oscillation can be counted repeatedly. A true shot can itself cause current/voltage changes, so the voltage guard can remove real shots. Source signals are held between logged changes; an unchanged value is not proof of a stale sensor, and a new log timestamp is not certified physical sensor freshness.

## Detector sensitivity: why a dip cannot yet mean one ball

| Recorded interval | Candidate dips at 5 / 10 / 20 rad/s | At 10 rad/s with voltage guard | Interpretation |
|---|---:|---:|---|
| Bensalem Q34 teleop | 136 / 118 / 108 | 0 | Large sensitivity to electrical filtering; do not report 118 shots. |
| Bensalem Q48 teleop | 33 / 21 / 17 | 0 | Far fewer feeding seconds, but still no calibrated conversion. |
| Bensalem Q53 teleop | 90 / 66 / 57 | 0 | Both shot loads and control/electrical response can create the pattern. |
| Tabernacle Q11 auto | 18 / 8 / 6 | 0 | Threshold choice alone changes the apparent count threefold. |

These are event-detector outputs, not lower/upper bounds on balls. Zero guarded candidates does **not** mean zero shots. The detector also does not enforce all device-validity/forward-feed checks needed for a production estimator. Six synthetic tests check the algorithm's stated rejection behavior, not real-world accuracy.

## What the motors do establish

Current retained code uses negative intake voltage for collection and positive for reverse. Historical dirty deployments/tunable settings require verification of that direction for each match. Here are recorded durations in the relevant direction, not ball counts:

| Interval | Negative intake >1 V (s) | Positive/reverse >1 V (s) | Kicker active with flywheel >100 rad/s (s) |
|---|---:|---:|---:|
| Bensalem Q34 teleop | 86.043 | 2.306 | 42.004 |
| Bensalem Q48 teleop | 49.590 | 15.986 | 14.220 |
| Bensalem Q53 teleop | 58.134 | 2.081 | 26.484 |
| Bensalem Q61 teleop | 39.667 | 3.258 | 6.325 |

Q48 has approximately 16 seconds of positive intake voltage in teleop, compared with 2.3 seconds in Q34. That is useful evidence for reviewing reversing/unjamming or driver control, **not proof of a jam**. The earlier absolute-voltage intake-active duration included both directions. Current and speed signals exist: Q34 teleop intake current spans 0–149.27 A and speed includes both signs. Those extrema do not identify how many balls crossed the intake.

Voltage is an actuator output, not a possession sensor. Current includes friction, acceleration, contact with the floor, packing and jams. A wide intake may acquire several balls together. The intake can run while feeding, touch the same ball multiple times, or run empty. Flywheel energy loss also includes changing speed commands, control ripple and supply effects; inertia, ball energy transfer and slip are not independently calibrated. Counting shaft rotations has the same problem unless one verified rotation corresponds to a metered ball, which has not been established here.

## A conditional volume calculation—not an empirical count estimate

If calibrated effective throughput during the measured feeding window were 1, 2 or 3 launched balls/s, Q34's 42.004 teleop feeding seconds would imply approximately **42, 84 or 126 launches**. Q48's 14.220 seconds would imply **14, 28 or 43**. This is `launches = measured feeding seconds × assumed effective rate`. The rates are illustrative; no available trial validates them. This is not a confidence interval, a supported range, or evidence that the actual answer lies between the endpoints. Effective rate must include empty feeding, jams and startup behavior. There is no responsible point estimate to choose among these scenarios yet.

Intake counts are even less identified. Inventory obeys `ending = starting + floor acquisitions + human-player additions − launches − ejections − losses`. Preload inventory, human-player supply, spills and end inventory are not all measured. Therefore even a future launch count does not determine how many balls the intake acquired. Season totals would additionally omit the matches and portions of matches with no usable log.

## Turn this into a useful estimator with the current hardware

1. Log a common timestamp, requested/applied feed state, flywheel goal/speed/current/voltage, kicker speed/current, intake speed/current/voltage, battery voltage, and device validity. Sample at the actual supported sensor rate; writing a 20 ms log does not increase the controller's underlying rate. Log pose/aim separately from ball detection.
2. Run counted trials: empty mechanisms; single balls spaced apart; known batches such as 5/10/20 balls; continuous feeding; reverse/unjam; drive acceleration with no balls. Use several battery states, shooting speeds, and intake surface conditions. Record initial/end inventory and actual launched count. Label the original timestamps from close-up video; field broadcast views may be insufficient.
3. Estimate the empty baseline and recovery signature at each operating point. Combine flywheel speed residual with kicker-current/speed disturbances and feed direction. Use refractory/hysteresis calibrated to real ball spacing, rather than adopting this probe's arbitrary values. Reject unknown/invalid telemetry explicitly. Consider continuous-load throughput regression if individual events overlap.
4. Split whole trials into calibration and held-out validation sets. Report bias, mean absolute count error, missed/double events and performance by voltage, speed and batch size. Do not split adjacent samples from the same trial into train/test; that would overstate generalization. A proposed initial acceptance target is error no worse than max(2 balls, 10% of the true count) on each held-out batch, with false positives in empty runs reported. This target is a team decision, not achieved performance.
5. Publish `EstimatedLaunched`, `EstimateValid`, `CalibrationRevision`, event confidence and reasons. Preserve candidate counts separately. Calibrate acquisition independently with counted floor intake trials and retained inventory. Never label launched fuel as scored fuel without observed outcomes.

A beam-break or other ball-passage sensor would improve observability, but that is a hardware change. The plan above uses the existing motors and camera/manual trial labeling. Historical estimates remain provisional even after calibration because old configurations, wear and battery behavior can differ.

## Reproduce and inspect

- Script: `robot-2026/scripts/ball_activity_probe.py` using the existing WPILib Python environment.
- Full source hashes, timestamps, settings and all 13 recordings: `ball-activity-probe.json`.
- All phase-level measurements: `ball-activity.csv` (Q11 has two separate enabled teleop windows; do not treat either as a full teleop).
- Reproduce: `robot-2026/.venv/bin/python robot-2026/scripts/ball_activity_probe.py`.
- Tests: `robot-2026/.venv/bin/python -m unittest discover -s robot-2026/scripts -p test_ball_activity_probe.py` — six passed.

The important immediate finding is a measurement limitation, not a discovered exact fuel count. The data does support prioritizing controlled shooting/feeding trials and investigating Q48's much longer reverse-intake activity.
