# Q11: power disturbance followed by network loss

The strongest supported account is a short controller-side power disturbance followed by a sustained loss of robot communications. The logs do not identify a failed connector. A separate, earlier problem is the absence of Module 1 drive feedback despite nonzero commands. It should be investigated independently rather than automatically blamed for the brownout.

The native review uses `logs/focus/Q11-electrical-focus.wpilog`, original FPGA seconds **410–425**, with no time rebasing. Its paired `videos/Q11_410-425_brownout.mov` starts at full-video second25.943. Frame1 is locked to log410.000. Video alignment uncertainty is approximately±1s, much larger than the brownout pulse durations. The clip is an observation aid, not a means of identifying a millisecond-scale electrical cause.

## What happened, in logged order

| FPGA seconds | Evidence | Interpretation |
| --- | --- | --- |
| Before417 | Module1 drive current, applied voltage, velocity and position remain zero for the entire recording. At415.993482 its optimized speed command is4.591m/s; its measured speed is0. Other modules have nonzero feedback. | An established command/feedback mismatch before the brownout. The recorded base code labels Module1 as front-right, CAN drive ID12. The dirty deployed working tree prevents treating that base mapping as fully verified. An unpowered/unreachable/misconfigured controller or an input/logging problem are candidates; zero feedback does not prove physical motor failure. |
| 417.513339–417.553555 | First BrownedOut=true pulse; SystemActive and6V Active go false and recover. | First recorded protection event in this incident. It is not proof that the processor rebooted. |
| 417.593364–417.633691 | Second pulse; controller input reports4.0357585V, below logged6.75V threshold. It reports12.3663V at417.613342.5V and3.3V active flags remain true. | Very short controller-input disturbance and recovery, with selective rail/output protection. |
| 417.613342–417.633691 | Raw PDH fault mask changes1048624→34603056→1048624; delta is bit25, named CAN-warning by the declared logger's packing scheme. | A coincident raw diagnostic change requiring logger verification. It does not locate a wire or establish a breaker trip. |
| 417.733435 | CommsDisableCount0→1; SystemActive stays false until475.138153. | A57.405s output-disable interval begins. Power voltage recovery does not mean robot control recovered. |
| 418.082935 | All three camera-connected flags become false. | Vision communication is unavailable. This does not mean three camera power circuits failed independently. |
| 419.436974 | Radio-status connected flag becomes false; status payload unavailable. | Radio reachability check fails. Status polling and timeout delays mean this is not necessarily when the radio physically lost power/link. |
| 422.605110 | DSAttached, FMSAttached and Enabled become false. | Driver Station/field attachment flags catch up later. These delays cannot establish a precise physical failure cascade. |
| 473.995065 | Radio becomes reachable. Reported6GHz rxBytes drop from8,760,055 at413.932707 to336; txBytes drop from279,590,768 to861. | Consistent with a radio restart or interface/counter reset. There is no radio uptime/boot-counter measurement here to distinguish those possibilities. |

The roboRIO continues producing cycle timestamps through this interval. The largest cycle-timestamp gap between410 and480s is0.128848s, not57s; the largest logged FullCycleMS during417–418s is23.299ms. A sustained roboRIO processor reboot or minute-long robot-code freeze is therefore poorly supported. This does not rule out a short disturbance or watchdog/communication effects.

## What the currents tell us

The robot had substantial earlier load: PDH total current is128A at the held sample for415s and138A near416s. Individual motor-controller current readings also show activity. These motor readings are controller-reported output currents, not interchangeable with PDH battery-side channel currents; do not sum or equate them to identify wiring.

Immediately around the controller minimum, PDH voltage samples bracket it at12.328125V at417.573348 and12.3515625V at417.633691, with total currents2A and4A. The PDH updates much more slowly than the controller protection flags. These samples argue against a **sustained** whole-robot high-current sag at the exact failure, but cannot exclude a short main-power transient between samples. They do not prove the disturbance was confined to the roboRIO branch.

## PDH data-quality correction

Q11 identifies base commit `f0e5fc88cf38aa3a792dafdaf0b42a1e975c38d6` and marks its working tree **uncommitted**. That commit declares AdvantageKit26.0.0. The upstream26.0.0 native PDH reader source contains suspicious channel indexing and fault-packing operations: channel3 is overwritten by reads intended for4/5; channels18–23 reference status2 data in the status3 block; multiple fault-mask updates use the existing mask in the combination where newly decoded bits would ordinarily be expected.

**Applicability to the deployed binary is unproven.** The recorded arrays do not consistently match every simple prediction of those source defects. A patched/rebuilt binary or another source/binary difference remains possible. We therefore preserve all24 recorded current values, highlight channels3–5 and18–23 amber for decoder review, and show raw PDH fault masks as warnings rather than certified physical faults. Do not conclude that channels4,5 or20 tripped merely because their bits appear in the baseline raw mask;1048624 was already present at13.949315s, long before this incident.

Before using those channels for wiring diagnosis, verify the actual deployed native library and compare against an independent, trusted PDH current/fault reading. Do not fabricate corrected historical channel currents by copying or shifting other channels.

Source: [AdvantageKit26.0.0 PDH reader](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.0/akit/src/main/native/cc/pdp_reader.cc), saved under `provenance/electrical-dashboard/akit_pdp_reader.cc`. [Recorded base dependency file](https://github.com/FRC1257/2026-Robot/blob/f0e5fc88cf38aa3a792dafdaf0b42a1e975c38d6/vendordeps/AdvantageKit.json). Both are source observations, not a verified deployed-binary inspection.

## Ranked potential causes

These are engineering hypotheses, not measured probabilities.

1. **Intermittent control-electronics power or ground connection.** The brief4.04V controller reading, continuing roboRIO logging and later network reset symptoms make a control-power feed, terminal, crimp, ground return or associated power hardware a strong investigation target. If radio and RIO feeds are independent, a fault only in the RIO positive lead would not explain a real radio restart by itself; inspect their shared upstream power/ground and the radio supply path too.
2. **A short interruption in the main battery/connector/breaker path.** A connector, cable termination or breaker connection can briefly interrupt power during robot motion. The slower PDH sampling could miss the lowest voltage, and earlier high loads provide context. A weak battery or excessive main-path resistance is possible, but the low nearby logged current means a sustained motor-load sag alone is not established.
3. **Radio supply/PoE/regulator or network-interface reset.** Loss of all camera connections and the radio traffic-counter reset support investigating the radio and its power/network path. A radio-only issue does not explain the controller4.04V observation; a shared power disturbance or a second issue would still be needed.
4. **A primary CAN-only fault or software stall.** Lower on current evidence. The PDH's raw CAN-warning bit merits examination, but the selected roboRIO CAN counters do not establish a leading bus failure, and a CAN-only explanation does not account well for controller undervoltage plus radio counter resets. The onset loop timing does not show a long robot-code stall.

Do not connect the preexisting Module1 anomaly causally to the brownout without further evidence. It could reduce available drive performance or reflect missing telemetry, but it does not by itself explain a controller power dip.

## Next discriminating checks

With robot power off, have the electrical subteam inspect and verify the battery/main-breaker connections, RIO power terminals, shared ground returns, radio supply and Ethernet/PoE connections. Compare RIO and PDH input voltage with independently trustworthy instrumentation during a controlled, supervised load/movement test. Confirm the actual PDH channel map and installed logger binary before interpreting channel faults. Check the Module1 drive controller's expected CAN ID, power, CAN reachability and vendor diagnostics; log per-device communication/error flags and bus voltage instead of relying on zero current as a connectivity test.

## Dashboard additions

- Electrical Map: all24 PDH channel-current bars,60A display scale; exact raw values remain available in three8-channel plots and the original array table. Amber indicates decoder review, not excessive current. Bars saturate visually; recorded values are not clipped.
- Motor Map:16 drive/steer/mechanism groups, reported current and voltage bars. Exact values, motor speeds and available temperatures have separate views. Missing current readings remain gray. Module1 zero feedback during peer motion is amber as an observation warning.
- Swerve Measured vs Setpoint: blue measured and orange optimized states in FL/FR/BL/BR order, with gyro heading.4.6m/s scale and0.6731m dimensions come from the recorded base code, not a new physical measurement.
- Mechanisms2D: measured hood and left/right hopper encoder angles versus goals. Blue is measured; purple is goal. Arm lengths and positions are schematic; encoder zero is not certified physical horizontal.
- Driver Inputs and Robot Alerts: recorded joystick axes/buttons and available alert arrays. Missing fields are not synthesized. Existing pose, trajectory, camera, tag and timing views remain available.

Reproduce the evidence with `robot-2026/.venv/bin/python robot-2026/scripts/analyze_q11_electrical.py`. The resulting `analysis/electrical/q11-diagnosis-evidence.json` retains exact source timestamps and a source hash. Radio exports are allowlisted to operational counters and signal/link state; raw authentication-bearing payloads are excluded. Native replay and paused screenshots are UI verification, not a completed full-match gameplay review.

## Cross-log follow-up (2026-10-16)

[Front-right swerve history](SWERVE_MODULE_HISTORY.md) finds the drive-feedback anomaly in March 12 practice logs and Tabernacle Q7, persisting throughout Q11 even after communications recover. Responsive feedback is present by Q53 the next day and in later usable logs. The exact physical failure and repair times remain unknown; an earlier January encoder-only episode is distinguished from March’s missing voltage/current/position/speed feedback.
