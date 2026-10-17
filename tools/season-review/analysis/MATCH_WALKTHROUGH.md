# Team 1257: synchronized match review and practical improvements

Prepared October 16, 2026. Start with **../Open Video Review.command**, or the running [local player](http://127.0.0.1:8826/?match=2026njtab_qm11). Select a match, play or scrub its video, and the telemetry follows. Click a chart to seek; chapter buttons jump to useful moments. The player works from downloaded local files and needs no YouTube login.

## What is ready

- All **136 supplied logs** were inspected: 124 decode normally, 3 yield readable prefixes before an error, and 9 are empty. Fifty-six have enabled windows, including practice. Per-file results remain in [log-analysis.json](log-analysis.json); the additional analysis is [season-metrics.json](season-metrics.json).
- TBA has **24 qualification appearances** and five playoff appearances for 1257 at the two supplied events. The archive has gameplay telemetry for **11 qualifications and two playoffs**, plus one pre-match-only qualification log. This is incomplete season coverage.
- **12 official FIRST Mid-Atlantic qualification videos** are downloaded, corresponding to every qualification with an identifiable supplied log. **11 are clock aligned**; Seneca Q31 has no gameplay alignment. Its recorded clock only takes pre-match values, with no enabled play. Its later EpochTime values also place the end before TBA's match actual_time, but SystemTimeValid was not established; no wall-clock alignment is substituted.
- [alignments.csv](video-sync/alignments.csv) and [manifest.json](video-sync/manifest.json) retain video/log hashes, sources, offsets, accepted/rejected clock anchors and later checks. The formula is `raw log seconds = video seconds + offset`. Estimated uncertainty is **±1 second**, not frame accuracy. Where later clock comparisons exist, disagreement with the corresponding one-second log clock interval is at most 0.39 s. Q58 has only early alignment support; Q53 and Q63 Seneca have only one later check before telemetry ends.
- The player shows no values outside the log extent. Disabled windows within a log remain visible. Missing signals display as missing. Graphs use 10 Hz held samples; exact transients and durations below come from original timestamped changes, not the display samples.
- The portable AdvantageScope layout now includes an **11 Match Video** tab. The native app successfully loaded and automatically locked the Q11 MP4 to its log. Other videos use saved alignments in the local player; loading another video in native AdvantageScope still requires selecting its MP4 and locking it. AdvantageScope 26's VideoController does not serialize the source or offset, so importing a layout alone cannot restore every video alignment. [Official Video instructions](https://docs.advantagescope.org/tab-reference/video/) explain playback and locking.

Raw MP4s, extractor metadata, frames and compact telemetry remain local and ignored by Git. Video copies total approximately 722 MB. Native AdvantageScope expands video into many JPEG frames and uses substantially more temporary space; the browser player streams the original MP4.

## First walkthrough: Seneca qualification 11

[Official recording](https://www.youtube.com/watch?v=durFirbjol0) · [TBA result](https://www.thebluealliance.com/match/2026njtab_qm11). Team 1257 is red with 1391 and 8130; blue includes 103, 190 and 7414. Final official alliance result is **123–193, loss**. This is an interesting reliability-and-recovery case, not a representative scoring average.

Use offset **384.057 s**. The chapter locations below have approximately ±1 s video uncertainty. Raw log transitions are more precise than that alignment.

| Video position | Raw log seconds | What to inspect |
| --- | --- | --- |
| 0–20 s | AUTO enabled 383.456–404.269 | The robot has 20.812 s of logged autonomous enable, intake activity 14.252 s, kicker activity 14.895 s and flywheel above 100 rad/s for 16.558 s. These show an operating mechanism sequence, not a scored-fuel count. The paused 15 s video frame shows red 17, blue 20. |
| 25 s | Teleop enabled from 408.280 | The broadcast shows 2:20 and red 23, blue 44. Start following voltage and DS attachment, rather than just the score. |
| About 33.5 s | BrownedOut true at 417.513–417.554 and 417.593–417.634 | Battery reaches 4.036 V during this first teleop window. Two brief brownout intervals total about 0.081 s. The sampled video shows 2:12 and red 23, blue 45. This proves a serious logged voltage event; the physical cause is unresolved. |
| About 38.5 s | 422.605 | Enabled, DSAttached and FMSAttached all become false. The DS clock had already stopped updating at 132, so it can disagree with the advancing broadcast countdown. Do not “correct” the video offset to follow that stale clock. |
| 65 s | About 449.057 | Still inside the recorded disabled/detached interval. The sampled broadcast shows 1:40 and red 65, blue 61. Partners and opponents continue affecting alliance totals; the score change cannot be assigned to 1257. |
| About 91.1 s | 475.138 | Enabled, DSAttached and FMSAttached return true after **52.533 s**. The DS clock resumes at 74. This is the useful recovery marker. Logs establish attachment and enable recovery, not exactly why communication stopped. |
| 96–150 s | Second enabled teleop window 475.138–548.614 | Intake activity returns for 27.077 s, kicker for 15.574 s, and flywheel above 100 rad/s for 27.464 s. Voltage remains below 9 V for 36.693 s of this 73.476 s window. Sampled video scores are red 78–blue 150 at 96 s, 104–160 at 120 s and 112–163 at 150 s; these are live overlay values, not final results. |

The main lost opportunity is an observed **52.5 s disabled/detached period**, preceded by severe voltage collapse. Electrical supply, radio/controller power, communication and robot logs should be investigated together. The evidence does not prove that a collision, battery connector, radio, breaker or software crash caused it. No reboot is inferred from the gap alone. The exact supporting changes are preserved in [q11-interruption-evidence.json](video-sync/q11-interruption-evidence.json).

Visual review was **18 paused frames across Q11, Q34 and Q42**, with no continuous full-match review. The [canonical observation record](../../research/observations/2026-1257/season-sampled-frames.json) separates visible overlay facts from telemetry and hypotheses. Individual shots, acquisitions, missed fuel and continuous cycle times were not counted. Small/occluded bumpers prevent reliable robot-level attribution in several frames.

## High points and low points supported by this archive

The official qualification alliance record improves from **3–9 at Seneca** to **7–4–1 at Bensalem**. This is encouraging team context, but partners, opponents and configuration changes prevent treating it as measured improvement in individual scoring. Bensalem Q2's 258–44 is the highest qualification alliance score in TBA's response, but its robot log is absent. Do not label it the best individual robot performance.

| Case | Evidence and interpretation | Practical use |
| --- | --- | --- |
| Bensalem Q34, win 148–73 | Full 140.014 s teleop: intake active 88.349 s, kicker 42.004 s, flywheel above 100 rad/s 63.115 s; flywheel connection remains true. Battery still spends 34.622 s below 9 V. | Best available long working comparison for mechanism activity. A useful baseline, not proof of 42 seconds of successful scoring. AUTO has zero kicker activity despite flywheel activity, so the win must not hide that auto question. |
| Bensalem Q7, win 233–48 | AUTO has 8.621 s kicker activity; recorded teleop has 23.097 s. Teleop coverage is only 69.683 s. | A strong alliance result with early mechanism operation. Do not compare whole-match productivity against full recordings. |
| Seneca Q63, win 145–140 | Close official result; log retains 20.656 s AUTO and only 41.463 s teleop, ending enabled. | Interesting competitive context, but the winning finish is outside robot telemetry. |
| Seneca Q11, loss 123–193 | 4.036 V event and 52.533 s disabled/DS-detached interval, then recovery. | Reliability improvement offers more usable match time than shaving small path segments. |
| Bensalem Q42, loss 63–88 | Full AUTO and teleop enable, yet drive applied voltage, intake, kicker and flywheel activity remain zero. Teleop driver translation input is nonzero for 24.384 s. CAN utilization is zero, mechanism connected flags are false throughout, TX errors range 64–248 and RX errors 128–132. PD current is zero; 12.66 V is unloaded and cannot establish a healthy robot. | Strong CAN/device-communication failure signature. Check power, wiring, bus connectivity and controller state; exact root cause is unknown. Use a pre-match diagnostic display and a short physical response check. |
| Bensalem Q48, loss 29–242 | Full teleop: 119.694 s drive-voltage activity, only 14.220 s kicker activity, 67.837 s below 9 V; minimum 6.982 V. Camera2 disconnected for 86.586 s. | More travel and demand did not produce more feeding time than Q34. Review route selection, shot setup and power demand; defense, partner traffic and ball supply remain unmeasured explanations. |
| Bensalem Q53, loss 47–221 | AUTO enable only 12.425 s; full teleop has 59.392 s below 9 V. | Investigate auto disable reason and power demand before increasing acceleration. |

Q34 and Q42 both log committed revision `5083ee80a875e8ab8ca3d9f7e376bda174516c6f` with **uncommitted changes**. Matching SHA does not establish identical deployed code. [CAN comparison evidence](video-sync/can-comparison-evidence.json) preserves the raw observed value ranges and connection flags.

## Strategy with the current robot

These are proposed experiments, not measured scoring gains:

1. **Protect match availability.** Before queueing, verify every required mechanism and drive controller responds, CAN errors are stable, all cameras have fresh data, and the loaded battery/connector path passes the team's physical checks. Q42 shows why a high idle voltage is insufficient; Q11 shows the cost of losing enable/communication. Log the check result and battery identifier to separate repeated hardware problems from control demand.
2. **Give the driver two or three calibrated stationary shooting locations.** Q34 has considerably more feeding time than Q48 while spending less time commanding drive. Use shorter collect–return–settle–shoot routes and agree on alliance lanes to reduce repeated cross-field travel. Review complete video and count actual release outcomes before promoting a particular route. This archive does not establish individual shooting accuracy or fuel per cycle.
3. **Plan collection around hub activity.** The 2026 rules alternate hub activity during alliance shifts; fuel in an inactive hub earns no fuel points. Collect, reload or reposition during inactive windows and be settled before the next active window. Both hubs are active during AUTO, transition and endgame. Add a clear active/inactive/unknown cue with time to the next transition, derived from FMS game data and the official schedule; if data is absent, show unknown. Some logs have no game-specific message, so historical wasted-window counts cannot be established here. Source: [2026 official manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf).
4. **Prefer one proven AUTO route with a reliable first release and bounded recovery.** Preload scoring plus a repeatable nearby collection/return is a better test baseline than a longer unverified route. Record route stages, selected variant, pose error, intake duration, first-ready time and first-feed time. Reject routes that never reach scoring or consume the entire auto waiting for a pivot. Alliance-specific lane conflicts and field starting positions must be checked in practice.
5. **Define an early endgame decision.** Agree with partners on tower access and a time budget before the match. Nothing in the sampled frames establishes climb success rates; use complete videos and explicit attempts before changing a climb policy.

## Software changes to prioritize

The following are findings in the downloaded source revision `f1a5fe50d65188cd047efaab31455f381f28a509`, not proof of what ran in every historical match. No performance-control code was changed or deployed during this review. Earlier local vision logging changes remain separate.

**1. Fix autonomous command completion before tuning speed.** In `AutoChooser.java:73`, `followPathFileCommand("RNZ_RS").alongWith(flywheel.runIdle())` waits for both commands. `runIdle()` calls the continuous `runEnd` velocity command in `Flywheel.java:149`; it never naturally finishes. Consequently, this right-trench sequence cannot advance to `targetedScore()` through normal completion. Make the path the deadline (`deadlineWith(flywheel.runIdle())`), then enter scoring. At `AutoChooser.java:80`, `hopperPivot.runIntakeAngle()` is another continuous `run` command, so `DEPOT_END` is unreachable through normal sequence completion. Add a measured at-goal condition and bounded timeout/fallback. These patterns also appear in committed log-era revision `5083ee8`, but dirty deployed builds prevent historical attribution. WPILib documents the [completion semantics](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html).

Acceptance: schedule the actual composed routine with fake subsystem I/O and a path-complete signal; assert scoring/depot-next-stage starts, interrupted idle stops, failed pivot reaches a safe fallback, and a healthy routine meets the AUTO time budget. Add stage telemetry so future logs make stalls obvious. Keep acceleration unchanged during this first experiment.

**2. Replace one-time shooter readiness with a deliberate feed policy.** `RobotContainer.java:262–279` waits for readiness but times out after 0.5 s and then feeds even if the predicate never became true; once feeding begins it does not keep checking readiness. Auto `targetedScore()` also gates only the initial feed and omits hub alignment in its predicate. During Q34 teleop, the kicker is active for **29.040 of 42.004 s while the logged flywheel-ready flag is false**; Q48 is **12.275 of 14.220 s**. These are timestamp-integrated overlaps, not missed-shot counts. The current 2.5 rad/s readiness tolerance may also be too strict under sustained load and needs calibration.

Use separate, logged “spin up,” “aligned,” “ready,” “feeding,” and “recovering” states. Select a tested velocity band, dwell and hysteresis for continuous feeding; pause or reduce feeding when speed/voltage falls outside that tested band, with a clearly indicated operator override. Avoid blindly stopping for every speed ripple. Acceptance: measure first-valid-feed latency, sustained measured fuel throughput and misses at the same distances and battery conditions; verify no automatic feed solely because a timeout expired.

**3. Add actionable device and power health telemetry.** Display disconnected devices, CAN errors, stale vision, loaded voltage and sensor freshness before enable. Log per-stage inhibit/recovery reasons and current-limited demand. Investigate wiring and power first for Q42/Q11; software cannot repair a disconnected CAN chain. Once hardware is sound, test demand coordination between drive, intake, agitation and shooter rather than increasing all demands together. Use controlled repeated trials to verify changes improve useful feed time without sacrificing drive control.

**4. Keep stationary targeting dependable before adding shoot-on-the-move.** The active targeted flywheel command uses `getStaticParameters`. `ShooterTrajectoryCalculator` has an unfinished moving-shot path, including repeated zero time-of-flight entries. Do not assume it provides validated lead compensation. First verify static distance/RPM/hood calibration, pose and camera freshness; only then implement measured flight time and velocity compensation with separate moving-shot trials. The new camera/tag logging supports diagnosis, but the reconstructed camera FOV remains a simulation assumption.

**5. Measure the bottleneck.** Add acquisition/possession sensing if available and explicit feed attempt/outcome signals; current motor commands do not count fuel. Add auto stage transitions, driver-request-to-ready latency, readiness reason, hub-active status and loop overrun context. These make future autonomous and driver comparisons quantifiable. Define success as validated fuel per active opportunity and usable enabled time, not higher average drive speed or alliance wins alone.

## Reproduce and verify

Install pinned Python requirements plus local `ffmpeg`, `ffprobe` and `tesseract`. Run `season_signals.py`, `video_clock.py`, then `build_video_sync.py` with `robot-2026/.venv/bin/python`. OCR uses retained frame crops at 2 Hz, plus later checks at 60/90/120/150/170 s. Invalid OCR offsets are rejected around the median. Local manual adjustments persist per match in browser storage and can be exported as JSON; they do not overwrite the source manifest.

Original official YouTube URLs are in [qual-video-urls.txt](video-sync/qual-video-urls.txt). Re-fetch with the pinned yt-dlp, no account cookies, with filenames `videos/%(id)s.%(ext)s`, ≤720p MP4 and `.info.json` metadata. Extractor metadata can contain expiring signed URLs, so it remains ignored. Sanitized source receipts and stable public URLs are in the manifest. The local server binds only to 127.0.0.1 and serves an allowlist with HTTP range seeking; its app-open action accepts only manifest matches from the local review origin.

Verification completed: 11 targeted Python review tests and 59 research pipeline tests pass. Repository validation passes with zero errors and four retained research warnings. KRAIL graph/source/integrity rebuild, doctor and cited search completed after updating the sampled-frame evidence count. Native video load/automatic lock was checked, then the app was restarted to clear its 6.2 GB temporary JPEG cache; the persistent browser review is left ready at Q11.

For a fresh checkout, the original TBA match response and receipt remain tracked under `robot-2026/provenance/tba/team/frc1257/matches/2026/`. Repository validation also registers them through an exact local mirror in ignored `data/raw/tba/team/frc1257/matches/2026/`; copy the response and `.receipt.json` there when restoring raw caches. This adds a provenance source, not a newly reviewed team-season.

## Electronics lesson: the controller can stop before the DS fields update

A deeper Q11 extraction on October 16 reveals a more precise sequence than the initial enable-only triage. The saved **1257-electronics-lab.json** layout adds the controller voltage, distribution voltage, actual brownout threshold, SystemActive, radio-status connection and 6 V rail state. Import it with File → Import Layout; reopen Q11 if import creates a new empty hub. Use View → Zoom to Enabled Range. The first chart is **1 Electronics Lab**, with numeric details in **12 Electronics Values**. Original season layout remains available separately.

### Read the chart

Blue is `/SystemStats/BatteryVoltage`, the controller-side voltage measurement. Green is `/PowerDistribution/Voltage`, a separate distribution-module measurement. Purple is `/SystemStats/BrownoutVoltage`, **6.75 V in this recorded log**. Orange is `/PowerDistribution/TotalCurrent` on the right-hand ampere axis. Do not read orange against the volt axis or use the controller input-current measurement as whole-robot motor current.

The discrete rows compare brownout, SystemActive, DS Enabled, DSAttached, radio Connected and 6 V rail Active. SystemActive reports FPGA output enable; it can go false because of disable/e-stop, watchdog expiry or brownout. CommsDisableCount specifically counts DS communication disables. Source: [WPILib RobotController API](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/RobotController.html). These are distinct measurements; a held DS Enabled=true is not proof that outputs remain available.

| Raw log s | Approx video s | Newly checked evidence |
| --- | --- | --- |
| 417.513339–417.553555 | 33.46–33.50 | BrownedOut=true, SystemActive=false, 6vRail/Active=false, then all three recover. |
| 417.593364–417.633691 | 33.54–33.58 | Another brownout/output/6 V interruption. The controller reports **4.0357585 V at 417.593364** and **12.3663147 V at 417.613342**. |
| 417.573348 and 417.633691 | 33.52 and 33.58 | Distribution voltage is **12.328125 V** and **12.3515625 V**; total current is **2 A** and **4 A**. These bracket the minimum controller reading but are not simultaneous high-speed measurements. |
| 417.733435 | 33.68 | SystemActive=false and CommsDisableCount increments 0→1. This begins a **57.405 s** uninterrupted SystemActive=false interval, ending 475.138153. It starts before DS Enabled/DSAttached update. |
| 418.082935 | 34.03 | All three camera-connected flags go false. This does not by itself prove their processors rebooted. |
| 419.436974 | 35.38 | RadioStatus/Connected becomes false. This is a connection-status observation, not a radio reboot reason. |
| 422.605110 | 38.55 | DSAttached, FMSAttached and DS Enabled become false. Their previously logged true values were insufficient to establish output availability during the preceding seconds. |
| 473.995065 | 89.94 | RadioStatus/Connected returns true. |
| 475.138153 | 91.08 | DS/FMS attachment, DS Enabled and SystemActive become true. |
| 475.838138–480.336123 | 91.78–96.28 | SystemActive becomes false again for **4.498 s** while the DS enabled window remains continuous; CommsDisableCount increments 1→2 at the start. “Enabled teleop seconds” therefore overstates output-available time in this window too. |

Video positions use the saved 384.057 s offset with ±1 s alignment uncertainty. Raw transitions are logged timestamps, not guaranteed exact physical transition times. BrownedOut totals about 80.5 ms across the two pulses. Source values are retained in [q11-electronics-signals.json](video-sync/q11-electronics-signals.json), with the original log hash in the synchronization manifest. The earlier **52.533 s DS-disabled/detached interval remains correct**, but SystemActive and the communication counter reveal an earlier and longer output interruption. Do not relabel that entire interval as a brownout.

### How the electronics work

A useful functional sketch, not a verified wiring schematic, is: battery → main breaker and distribution → individual motor-controller branches and a controller/radio power branch. Control information travels Driver Station → radio/network → roboRIO → CAN/PWM motor controllers. Power and communication are separate paths: a powered controller can still inhibit outputs if it loses DS communication.

Motors draw current. Battery internal resistance, wires, crimps, terminals and connectors all create voltage loss under load: approximately `V_load = V_open − I × R_path`. A weak battery, high simultaneous demand, or extra connection resistance can each lower the available voltage. Example only: an added 0.010 ohm at 100 A causes 1 V of drop; that is not a measurement of this robot. An intermittent open or branch supply problem can produce a different pattern from ordinary whole-robot load sag.

Brownout protection reduces/disables outputs to preserve control-system power. A voltage recovery after disabling loads can therefore coexist with a stopped robot. The recorded 6.75 V setting matters more than a remembered universal threshold: roboRIO generations/settings differ. See [WPILib brownout guidance](https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-brownouts.html). Q11 specifically logs the 6 V rail inactive during both brownout pulses; its logged 5 V and 3.3 V Active flags remain true. Do not claim all sensor rails or the processor shut down.

The controller-side 4.04 V reading versus near-12.3 V distribution readings is a useful reason to investigate the controller power branch, connector/return path, transient response and sensing. **It does not prove a localized wiring fault.** Distribution samples around this event are roughly 60–80 ms apart while the lowest controller sample lasts about 20 ms; a brief common supply dip could be missed by the slower source. Stored/filtered readings and sensor timing also matter. Likewise, the 2 A distribution reading does not prove that no short high-current pulse occurred. There is insufficient synchronized high-speed evidence to identify the exact failed component.

The log continues recording through the interruption, which does not support simply calling the entire interval a roboRIO reboot. A radio power/network interruption is a hypothesis to test, not an established cause. The communications-disable count and SystemActive are much stronger evidence for the loss of output availability than the DS enabled flag alone.

### A repeatable electronics-subteam investigation

1. Open the original Q11 log and the lab layout. Start with the full match: find the sharp voltage notch around 417.6 s and the long low-current plateau afterward. The plateau means recorded electrical demand dropped, not that the robot had a healthy match.
2. Hover the notch and zoom horizontally using the trackpad/wheel. Inspect roughly 416–424 s, then zoom further to 417.45–417.80 s. Click to retain a time, and use Left/Right to step cycles. Confirm the brownout pulses and compare voltage source timestamps in Electronics Values; do not trust the visual width of a pulse at full-match zoom.
3. Compare SystemActive with Enabled. At 417.733 s the former goes false first. Verify the CommsDisableCount increment in the values table. Follow radio and DS flags through recovery near 474–475 s, then inspect the second interruption near 475.838–480.336 s.
4. Move to Timing + Sensors at the same shared cursor and compare camera and CAN behavior. A camera disconnection can mean power loss, network loss, processing delay or stale telemetry. A motor command or current reading alone is not a complete health diagnosis.
5. Record three columns: what changed, what might explain it, and what measurement would distinguish those explanations. For this case: compare battery/distribution/controller/radio input voltage with suitable synchronized instrumentation; correlate with DS `.dslog/.dsevents`, radio logs/uptime and device reset counters. Match all clocks before comparing.
6. With the robot powered off and under the team's normal mentor procedures, inspect battery connections, breaker/distribution terminations, controller/radio branch connections, return wiring and strain relief. Use manufacturer specifications for terminals and approved protection. A powered load test should be controlled and supervised; do not bypass brownout protection or increase breaker ratings to hide the symptom.
7. Change one verified issue at a time, repeat the same controlled load and log the battery identity and wiring change. A successful check establishes what was tested; one clean run does not prove every possible intermittent fault is gone.

A useful first exercise: explain why SystemActive=false at 420 s even though the stored DS Enabled field is still true, and why the 12.3 V distribution reading does not disprove a 20 ms voltage dip at the controller. Answering those two questions demonstrates the distinction between control state, sensor location and sampling rate.


### Q11 focused native video review

The local derivative `logs/focus/Q11_brownout_focus_ZERO_EQUALS_410s.wpilog` limits the view to original FPGA time 410–425 seconds. Displayed time zero equals original 410 seconds. All original signal payloads are preserved except the AdvantageKit `/Timestamp`, which is shifted with record timestamps. Held values at the boundary are explicitly synthesized; these are viewing aids, not new measurements. The original season log is untouched. Preservation checks and hashes are in `analysis/video-sync/q11-focus-receipt.json`.

Load `videos/Q11_410-425_brownout.mov` into the Match Video tab. With selected log time 0 and video frame 1, click the lock. Use the Table timestamp entry (click it, paste the time, press Return) to select an exact time. At displayed 7.500 seconds, native AdvantageScope showed video frame 226 and the broadcast clock 2:12. The clip starts at full-match video 25.943 seconds; clock-derived alignment uncertainty remains approximately ±1 second. Video cannot resolve which event occurred first within the roughly 40-millisecond brownout pulses.

Start playback around displayed 3–5 seconds to see the lead-in. First brownout starts at 7.513339; minimum controller voltage 4.0357585 V is at 7.593364; system output enable falls and CommsDisableCount increments at 7.733435. Radio connected falls at 9.436974; DSAttached falls at 12.605110. These flags are distinct stages, not evidence of one simultaneous reboot.

Native UI limitation: the separate video satellite remained on its first frame. Use the main Match Video tab, where frame 226 at selected 7.500 seconds was verified. Closing the stale satellite subsequently caused the native UI control service to time out.
