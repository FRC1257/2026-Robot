# Team 1257 — 2026 season log review

Automated telemetry triage prepared October 16, 2026. AI review covers 18 paused frames across three qualifications; no full match has been continuously reviewed. See MATCH_WALKTHROUGH.md for synchronized videos, bounded observations and strategy/software recommendations.

Inspected all 136 supplied files: 124 decoded without a reported exception, 3 retained decoded prefixes with a decode error, and 9 empty files. The native reader can also stop at truncated trailing records without raising; a successful decode does not certify byte-level completeness.
TBA lists 29 match appearances across two events; 13 have enabled telemetry in FMS-identified match logs in this archive. Other match logs are absent or contain only disabled data. Practice logs are analyzed separately in log-analysis.json.

## How to walk through a match

1. Double-click `../Open Season Review.command`. It opens Bensalem Q48 in AdvantageScope. To change matches while keeping the layout, use File → Open Log(s), or one of the launchers in `../matches/`.
2. Start in **1 Match + Battery**. Blue is battery volts, orange is total current. The discrete rows mark Enabled, Autonomous, and BrownedOut. Click a time in the graph, then press Space to play/pause. Left/Right step by recorded cycle while paused. Right-click Play for slower playback.
3. Switch to **2 Robot 3D** to see the same selected time on the 2026 field. The supplied approximate model and intake/hood component pose array are connected. The model is illustrative; its alignment is not CAD validated.
4. Use **3 Field + Auto** for estimated robot pose versus the trajectory setpoint. Pose corrections and resets can look like robot jumps; they are not proof of physical motion.
5. Use **4 Shooter** to compare measured angular velocity with its goal, readiness, and kicker command. Early-season logs use FlywheelAngularVelocity; later ones use FlywheelLeaderAngularVelocity. One may be absent. A spinning flywheel or kicker command is not a counted scored ball.
6. Use **5 Intake + Hood** for hood/pivot angles and intake/kicker voltages, and **6 Timing + Sensors** for loop duration, CAN use, gyro and camera connection flags. Missing signals remain missing, not zero.
7. Command+Left / Command+Right switch tabs. For another window or a changed layout, use File → Import Layout and choose `1257-season-review.json`. The layout is portable within AdvantageScope 26.0.2; the custom model must also be installed.

## Matches worth reviewing first

Times below are raw WPILOG timestamps in seconds (FPGA time since boot), not video time or match countdown. For these unmerged logs, AdvantageScope uses the recorded timestamp directly; use the raw times below. A multi-log merge or manual time alignment can change the displayed reference. Window boundaries may include scheduler/DS update latency.

### 2026njtab_qm11 — alliance loss, 123–193

Investigate the 4.04 V logged dip and the split enabled periods. Check DS connection, brownout, loop time and video before assigning a cause.

Log: `akit_26-03-14_17-33-19_njtab_q11.wpilog`; source `wpilog:sha256:f214b527dead1344b5282ca99dbf0c153d4258fa56072db0317db85d80a554b9`; first record **2.764 s**.

- Raw 383.456–404.269 s; enabled 20.812 s; autonomous 20.812 s; min battery 8.641 V; below 9 V 0.182 s; logged brownout 0 s; intake commanded 14.252 s; kicker commanded 14.895 s.
- Raw 408.280–422.605 s; enabled 14.325 s; autonomous 0 s; min battery 4.036 V; below 9 V 0.762 s; logged brownout 0.081 s; intake commanded 0.286 s; kicker commanded 0 s.
- Raw 475.138–548.614 s; enabled 73.476 s; autonomous 0 s; min battery 7.497 V; below 9 V 36.693 s; logged brownout 0 s; intake commanded 27.077 s; kicker commanded 15.574 s.

[TBA match](https://www.thebluealliance.com/match/2026njtab_qm11) · 10 paused frames reviewed by AI; no continuous match review.

### 2026paben_qm48 — alliance loss, 29–242

Start here for sustained low-voltage review. Compare battery with current, drivetrain demand, and flywheel recovery. Alliance score is not a measurement of 1257 scoring.

Log: `akit_26-03-29_13-40-03_paben_q48.wpilog`; source `wpilog:sha256:90206ca85086f42e03f9add646280a5f2ae95177fcc33208b09654981f3d1067`; first record **1.642 s**.

- Raw 135.035–155.829 s; enabled 20.795 s; autonomous 20.795 s; min battery 8.572 V; below 9 V 0.34 s; logged brownout 0 s; intake commanded 19.072 s; kicker commanded 10.314 s.
- Raw 159.894–299.940 s; enabled 140.046 s; autonomous 0 s; min battery 6.982 V; below 9 V 67.837 s; logged brownout 0 s; intake commanded 65.575 s; kicker commanded 14.22 s.

[TBA match](https://www.thebluealliance.com/match/2026paben_qm48) · video remains unviewed.

### 2026paben_qm42 — alliance loss, 63–88

Contrast case: unusually flat battery voltage. Check whether the robot actually moves, whether motors receive commands, and whether telemetry is stale before calling this healthy operation.

Log: `akit_26-03-28_22-29-34_paben_q42.wpilog`; source `wpilog:sha256:a575a400bd5ac588b0678b9d4dcf484b21c6ac0bb4a85025eff35ce057c83ebd`; first record **2.662 s**.

- Raw 135.993–156.190 s; enabled 20.197 s; autonomous 20.197 s; min battery 12.668 V; below 9 V 0 s; logged brownout 0 s; intake commanded 0 s; kicker commanded 0 s.
- Raw 160.210–300.240 s; enabled 140.030 s; autonomous 0 s; min battery 12.662 V; below 9 V 0 s; logged brownout 0 s; intake commanded 0 s; kicker commanded 0 s.

[TBA match](https://www.thebluealliance.com/match/2026paben_qm42) · 4 paused frames reviewed by AI; no continuous match review.

### 2026paben_qm34 — alliance win, 148–73

Winning alliance comparison with long enabled telemetry. Use it as a comparison for Q48 rather than treating the win as evidence of a particular mechanism advantage.

Log: `akit_26-03-28_21-18-11_paben_q34.wpilog`; source `wpilog:sha256:ab548007210f84a644fbd49f62bdf42d3acf0baa84f8deb84ace8d3c02f7086f`; first record **2.801 s**.

- Raw 129.734–150.449 s; enabled 20.715 s; autonomous 20.715 s; min battery 8.39 V; below 9 V 0.673 s; logged brownout 0 s; intake commanded 18.042 s; kicker commanded 0 s.
- Raw 154.450–294.465 s; enabled 140.014 s; autonomous 0 s; min battery 7.384 V; below 9 V 34.622 s; logged brownout 0 s; intake commanded 88.349 s; kicker commanded 42.004 s.

[TBA match](https://www.thebluealliance.com/match/2026paben_qm34) · 4 paused frames reviewed by AI; no continuous match review.

### 2026paben_qm53 — alliance loss, 47–221

Autonomous enabled period is shorter than the ~20 s segments in most other logs. Investigate enable/disable transitions; the log alone does not establish why.

Log: `akit_26-03-29_14-27-29_paben_q53.wpilog`; source `wpilog:sha256:f1ecb2dfd0cb08514d925ed20d7ef12480e8e9e2bf4ab68548e35f848b099b55`; first record **2.536 s**.

- Raw 248.003–260.428 s; enabled 12.425 s; autonomous 12.425 s; min battery 7.686 V; below 9 V 1.4 s; logged brownout 0 s; intake commanded 10.563 s; kicker commanded 2.254 s.
- Raw 272.450–412.530 s; enabled 140.080 s; autonomous 0 s; min battery 7.07 V; below 9 V 59.392 s; logged brownout 0 s; intake commanded 60.215 s; kicker commanded 26.484 s.

[TBA match](https://www.thebluealliance.com/match/2026paben_qm53) · video remains unviewed.

### 2026njtab_qm63 — alliance win, 145–140

Close alliance win (145–140) with limited enabled telemetry. Good video follow-up; do not equate the short log segment to the full match.

Log: `akit_26-03-15_16-37-51_njtab_q63.wpilog`; source `wpilog:sha256:b4e02b0781d9503736e60348bdd1cf1d77648f66ad037db51c88e4519dfa5dda`; first record **2.381 s**.

- Raw 179.274–199.929 s; enabled 20.656 s; autonomous 20.656 s; min battery 8.427 V; below 9 V 0.16 s; logged brownout 0 s; intake commanded 18.936 s; kicker commanded 8.842 s.
- Raw 203.949–245.412 s; enabled 41.463 s; autonomous 0 s; min battery 7.491 V; below 9 V 10.226 s; logged brownout 0 s; intake commanded 25.175 s; kicker commanded 4.2 s. **Log ends while enabled.**

[TBA match](https://www.thebluealliance.com/match/2026njtab_qm63) · video remains unviewed.

### 2026paben_sf1m1 — alliance loss, 184–384

Playoff comparison with a long teleop record. Compare pre-match battery, demand, and subsystem readiness against qualification runs.

Log: `akit_26-03-29_18-53-11_paben_e1.wpilog`; source `wpilog:sha256:00b6aad7e16093514fc878de31e9c37bee41eb14f1a9af11dee7cd1665f13112`; first record **1.646 s**.

- Raw 229.151–250.304 s; enabled 21.153 s; autonomous 21.153 s; min battery 8.559 V; below 9 V 0.22 s; logged brownout 0 s; intake commanded 19.658 s; kicker commanded 10.859 s.
- Raw 254.403–395.542 s; enabled 141.138 s; autonomous 0 s; min battery 7.271 V; below 9 V 36.516 s; logged brownout 0 s; intake commanded 74.41 s; kicker commanded 31.305 s.

[TBA match](https://www.thebluealliance.com/match/2026paben_sf1m1) · video remains unviewed.

### 2026paben_sf5m1 — alliance loss, 127–162

Short playoff enabled record. Determine whether logging stopped, the robot disconnected, or the robot was disabled; these remain competing hypotheses.

Log: `akit_26-03-29_19-37-09_paben_e5.wpilog`; source `wpilog:sha256:9803d5bf6cdcde9b098a4fe988cbd7630e46265c94949b8e91f2f93cf96d456e`; first record **4.154 s**.

- Raw 89.366–109.765 s; enabled 20.400 s; autonomous 20.4 s; min battery 8.346 V; below 9 V 0.08 s; logged brownout 0 s; intake commanded 18.882 s; kicker commanded 10.227 s.
- Raw 113.788–146.961 s; enabled 33.173 s; autonomous 0 s; min battery 7.837 V; below 9 V 3.441 s; logged brownout 0 s; intake commanded 20.113 s; kicker commanded 0 s. **Log ends while enabled.**

[TBA match](https://www.thebluealliance.com/match/2026paben_sf5m1) · video remains unviewed.

## Evidence and limits

TBA result source: `tba:team/frc1257/matches/2026:bba8c68c7525a40c`, retrieved `2026-10-16T22:53:09Z`; raw response and receipt under `../provenance/tba/team/frc1257/matches/2026/`. Scores are alliance outcomes. The playoff eN → sfNm1 mapping was checked against this team’s recorded lineup appearances.
Raw ZIPs remain in Downloads; extracted originals remain in logs/ and models/. Each analyzed file has a SHA-256 and byte count in log-analysis.json, including empty/error files. Source archives have receipts in provenance/receipts.json.
Enabled durations and below-threshold durations use sample-and-hold integration, not sample percentages. A 9 V threshold is an exploratory diagnostic choice, not the roboRIO brownout threshold. BrownedOut is reported separately. Missing signals return null. No extrapolation is made beyond the log end. Window metadata contains the values held at the window end: Enabled may read false, and FMSAttached may fall false at a disconnect. Such periods are retained when the event/match identity and log-level FMS evidence match.
Odometry travel is an estimate; steps above 8 m/s are omitted and recorded as pose jumps. Smaller estimator corrections can still inflate travel, so do not rank driving quality using this number.
Log build GitSHA/dirty metadata is retained. Current cloned source is a later revision and is not automatically the code that generated each log. Full-season coverage, motor physics, camera accuracy, exact geometry and actual ball counts are not established by this analysis.

## Decode gaps

- `akit_01f9229c88fbd66f.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_0714c98e6792f31b.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_17d6115cc61da9a3.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_330c482682ef4ab5.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_4468fa60b2ba38ff.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_7335396034f3d446.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_739c9e541d976530.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_d3fe4a9d3e713470.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_d63bd7386825b745.wpilog`: Invalid WPILOG header; 0 bytes. No usable decoded record.
- `akit_26-01-24_16-22-15.wpilog`: not a start record; 5111808 bytes. Only its decoded prefix is usable.
- `akit_26-01-28_20-49-34.wpilog`: not a boolean; 2654208 bytes. Only its decoded prefix is usable.
- `akit_26-03-12_22-37-01.wpilog`: not a start record; 156631040 bytes. Only its decoded prefix is usable.
