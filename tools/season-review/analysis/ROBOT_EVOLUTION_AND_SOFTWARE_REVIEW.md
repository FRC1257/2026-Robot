# Robot evolution and software priorities with the same hardware

Review date: October 16, 2026. This is a retrospective of the supplied logs and locally available Git history, not a measured estimate of additional points. No robot code was modified for this review.

The robot evolved from an inherited swerve platform into a coordinated hopper/shooter robot with distance-based aiming, multiple autonomous routines, and driver alignment aids. The strongest improvement opportunity is making those pieces cooperate reliably: finite autonomous stages, continuously checked shooting readiness, power-aware demand, and trustworthy device health. A higher top-speed setting would not address the most substantial evidence here.

## Evidence boundaries

- Logs cover January–March 29; available branch history extends into May. Later commits are intentions/implementation evidence, not proven deployed improvements.
- Relevant competition builds report uncommitted changes. A log's GitSHA identifies a base, not its exact executable source. Static bugs below are proven properties of retained code, not automatic explanations for individual matches.
- Git history includes inherited 2025 code and multiple branches. This review focuses on 2026 changes and separates later branch work from the March competition builds.
- Source snapshots, full revision hashes, patches, source URLs, retrieval times and SHA-256 receipts are in `../provenance/software-evolution/`. Log-source hashes and bounded metrics are in `software-evolution-evidence.json`; original log inventory and the cross-log swerve study remain separate evidence.
- Kicker voltage is a command/output observation, not a ball count. A false readiness flag does not prove a miss. Low-voltage and connectivity observations do not identify a bad wire.

## How the robot changed

| Period | Code evidence | What telemetry establishes | Interpretation and remaining gap |
| --- | --- | --- | --- |
| January: inherited drive platform | Jan 13 `e14861b` imports the previous platform; Jan 21 `4e7c726` updates CAN IDs. | January tests have changing four-module feedback. January 28 has an encoder-only front-right anomaly, followed by recovery that day. | Basic driving was available early. That encoder-only episode differs from March's absent electrical and encoder feedback. |
| February: mechanisms and test infrastructure | Hood/flywheel/kicker/hopper implementations, simulations, late-February rewrites and tuning utilities (`45b3940` adds tunable numbers). | February 25 has an extended practice run with responsive front-right feedback; the archive also has many disabled/short tests. | The software grew from separate mechanism bring-up toward shared control. Commit titles do not certify mechanisms physically worked. |
| March 7–13: geometry, shooting and autonomous integration | `ad5d9ba` changes three steering offsets and changes encoder offset addition to subtraction. March 8–12 adds/adjusts hub aiming, shooting tables and autos; `1760063` is the March 12 logged base, `f0e5fc8` the first-event base. | March 12 practice already has persistent front-right drive feedback loss. Tabernacle Q7/Q11 retain it. | The March 7 swerve fix concerns steering coordinate calibration; it did not prevent the later drive-controller telemetry anomaly. These should not be called the same bug. |
| March 15: first-event adjustments | `f085965` and `83e550c`, merged in `4e45023`, modify controls, paths and shooting calibration. Lookup-table distance keys shift outward by 0.3429 m, changing which speed/hood value is requested for a pose. A 1.5 s teleop readiness timeout is introduced in the merged code. | Q53 has responsive front-right speed/position/current/voltage; Q58/Q63 also do. | Feedback availability recovered, but the committed drive files are unchanged between Q11/Q53. No repair record or exact dirty source explains the recovery. The table shift is a calibration change, not proof earlier shots were short/long. |
| March 16–23: reliability, load and mechanism changes | `fd60dd9` reduces six mechanism current limits to 40 A. Later commits revise limits again. `7ea645a` removes climb software and adds energy logging; `01c9126` adds subsystem connection alerts/telemetry; `975d4a7` adds a second hopper-intake motor in software. Hood/pivot/voltage clamps are added. | Later logs expose more device telemetry, making earlier/later availability comparisons uneven. | These are concrete attempts to address robustness and mechanism operation. Removing climb code does not establish mechanical removal or failure. Two-motor support suggests hardware/configuration evolution but is not a physical inspection. |
| March 18–23: vision experiments | Limelight-related work appears, followed by `a10f56f` returning toward PhotonVision; `aa2546d` changes drive/vision integration. | Camera connectivity and estimator corrections are visible, but complete accepted/rejected observation histories were not logged. | Vision architecture was being iterated. Connection flags are insufficient to rank pose accuracy or conclude a camera was physically broken. |
| March 21–29: faster feeding and richer driver/auto behavior | `3189eff` removes the 1.5 s timeout; `01c9126` reinstates a shorter 0.5 s timeout. `9004231`/`6b19b9d` revise point-and-lock behavior. `4517e0c` adds paths and sweep/outpost options; `90560c0` adds idle spin during a return path. `5083ee8` adds both trench headings. `eaa6f81` waits for hub alignment before selecting lock/drive and adds an alignment predicate to feeding. | Bensalem has responsive front-right feedback in usable runs, but repeated low-voltage operation and feeding with readiness false. Q34 has no kicker output during autonomous; Q48 later records autonomous kicker output on the same sweep selection. | More capability did not eliminate coordination/tuning constraints. Q34 versus Q48 is suggestive of readiness/timing variability, not a clean before/after experiment. |
| After the supplied competition logs | March 31 `192680a` introduces Paralib work; April adds tunable persistence/periodic changes; May 26 `4b7ed85` refactors vision toward Limelight/PhotonVision IO on a later branch. | No supplied match telemetry validates those later changes. | Describe them as subsequent development, not demonstrated season recovery or scoring improvement. |

## What appears to have broken or limited operation

**Front-right drive feedback:** January's encoder-only episode recovers that day. The March episode is present in practice, Q7 and all of Q11, then responsive by Q53. Missing speed, position, current and output voltage supports checking device power/CAN/configuration/read validity; it does not prove a motor mechanically stopped. See `SWERVE_MODULE_HISTORY.md` for exact file/time evidence.

**Q11 control power/network interruption:** the RIO records about 4.04 V and two brief brownout intervals around FPGA 417.5–417.6 s, followed by a long outputs/communications interruption. Continuous robot logging argues against a minute-long program freeze. The persistent front-right issue predates this event and survives communication recovery. Software cannot repair an intermittent power connection; it can expose the fault sooner and improve controlled recovery. See `Q11_FAILURE_ANALYSIS.md` for ranked electrical hypotheses.

**Power headroom later in the season:** Bensalem Q34 has 34.622 of 140.014 recorded teleop seconds below 9 V; Q48 has 67.837 of 140.046; Q53 has 59.392 of 140.080. These are RIO battery-input observations using an exploratory threshold, not extra brownout flags. Battery condition, wiring resistance and demand remain competing explanations. Current-limit changes show active tuning, but there is no controlled evidence that one commit fixed power behavior.

**Shot readiness and feeding:** Q34 teleop has 42.004 s kicker active, including 29.040 s with flywheel readiness false. Q48 has 14.220/12.275 s respectively; Q53 has 26.484/22.126 s. This can include wheel slowdown after a ball enters, an overly tight readiness tolerance, operator override, or a command that keeps feeding after initial readiness. It does not identify misses. The recorded base uses a ±2.5 rad/s flywheel readiness tolerance; making this arbitrarily wider would hide evidence rather than establish acceptable shots.

**Q42 telemetry/system health:** all four drive modules are near zero, several fields are implausible, the flywheel connection flag is false, and recorded intake/kicker activity is zero. It is not a healthy low-current comparison. The evidence does not isolate a single failed component or show whether the entire physical robot was inactive.

## The first software changes I would make

### 1. Give autonomous stages explicit completion and failure behavior

This is the clearest static correctness opportunity. In the Bensalem base:

- RIGHT_TRENCH chains `followPathFileCommand("RNZ_RS").alongWith(flywheel.runIdle())` before scoring. `runIdle()` is a continuous command, so the parallel group does not normally finish when the path finishes. Use a path deadline so the spin-up command ends with the path and scoring can start.
- DEPOT places continuous `hopperPivot.runIntakeAngle()` between the first scoring timeout and `DEPOT_END`. Without an end condition, the later path is unreachable in that source. Make positioning a finite, bounded stage and hold the mechanism through an appropriate parallel/default control afterward.
- An initial `.until(atGoal())` can wait indefinitely when feedback never reaches the predicate. Give positioning/alignment bounded success/failure outcomes; on failure choose a conservative fallback rather than silently advancing as though successful.

These conclusions follow WPILib's composition semantics: `alongWith` waits for all members, whereas a deadline ends its companions when the deadline command ends. [Official WPILib documentation](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html).

Do not claim these exact bugs caused a particular match's failed auto. Some RIGHT_TRENCH logs record autonomous feeding despite the committed base's blocking construction, which is further reason to inspect the actual dirty deployment. No command-stage transition trace exists to resolve the discrepancy.

Verification: run every chooser option in scheduler simulation, assert that intended stage transitions occur, inject missing position feedback and path failure, and assert the fallback. Log selected auto and every stage's enter/exit/reason. This has a clearer acceptance condition than judging an auto by its name or an alliance score.

### 2. Replace the shooting expression with a continuously evaluated shot coordinator

This is my highest expected scoring-throughput improvement, provided the robot is electrically healthy. The teleop expression `waitUntil(ready).withTimeout(0.5).andThen(feed)` advances after either readiness OR timeout; the timeout is not a successful readiness check. Once feeding starts, readiness is not continuously reapplied by that expression. The earlier Q11 base also waits only once before feeding, even without the timeout.

Use explicit idle, spin-up/aim, ready, feed, recover and fault states. Compute one coherent distance/hood/speed/heading solution per cycle. Require valid device feedback and a stable measured speed/hood/heading before normal feeding; pause or reduce feeding when speed falls below a separately calibrated recovery boundary. Use hysteresis to avoid oscillation. Include target validity/range, pose confidence and suitable chassis speed. Provide a deliberate, logged driver override if desired; a spin-up timeout should report inability to become ready, not silently impersonate success.

Combine this with measured flywheel feedforward/PID tuning and empirical shot-table calibration at repeatable distances. Pre-spin only when a shot is likely and power headroom permits. Existing command architecture has the ingredients; coordinating them is the missing piece. Without a ball sensor or manually labeled video, commanded feeding time is not actual throughput, so validate with counted successful shots and recovery time on the same hardware.

Also fix wrapped heading error in `isHubAligned()`: simple subtraction can report a nearly full-turn error across the ±π boundary. Review the one-time `Commands.either` choice after alignment; it is not a continuously reevaluated transition between locked and driving states. These are source-level corner cases, not measured causes of season losses.

### 3. Manage combined power demand

Prefer bounded acceleration, smooth flywheel ramps and coordinated intake/agitation/feed demand to blanket increases in current limits. Preserve steering/control authority and shot stability when voltage falls; restore limits with hysteresis rather than rapid switching. Tune under representative multi-mechanism load, not only one subsystem at a time. Account for the fact that reduced acceleration or idle spin can trade energy against cycle time; measure it.

Use trusted battery-side current and voltage for the power model. Controller motor currents cannot simply be summed and multiplied by battery voltage to obtain battery power under PWM operation. The existing EnergyLogger does that approximation, assumes 0.5 A for the radio, and uses fixed cycle time. Treat it as an estimate, or replace it with calibrated battery-side measurements and measured elapsed time. The previously identified PDH source-quality concerns must be resolved before using that stream as a closed-loop authority.

### 4. Make device failures impossible to mistake for valid zeros

Extend the March alert work to each swerve drive and steering controller. Log last successful update, individual read/configuration/setpoint errors, bus voltage, reset/fault information, device identity and configuration revision. Alert clearly when commanded motion plus moving peers produces invalid or persistently zero feedback; avoid declaring a failure from a normal stationary wheel.

Before each match, a deliberate supervised preflight should confirm all four drive/steering feedback paths, gyro, shooting devices and camera freshness. Do not automatically spin motors on enable or mask a fault with invented measurements. A driver-visible health summary and a logged override are more useful than blindly allowing full-performance autonomous operation with unknown odometry.

### 5. Make localization and deployment repeatable

The PhotonVision implementation returns cached results when no new unread frames arrive. The drive layer can reconsider these at old timestamps without an explicit per-camera consumed-timestamp gate. Process each observation once, reject stale/out-of-order/implausible estimates, and log observation age, tag IDs, ambiguity, uncertainty and acceptance/rejection reason. Fix bounds validation before indexing vision arrays. Evaluate against replay scenarios and known field measurements; do not infer accuracy from camera connectivity.

Save exact deployed source/configuration, controller IDs, camera transforms, path hashes and shot-table revision with each log. Dirty-build metadata made several otherwise promising comparisons impossible. This is a direct debugging/time-to-repair improvement even when it does not change a control loop.

## Practical order and success criteria

First address known electrical faults and establish trustworthy four-module feedback. Then fix finite auto stages and introduce the shot coordinator; validate both with power-demand limits. After that, optimize paths and localization. Keep the current mechanical design throughout this comparison.

Measure: completed intended auto stages; shots made/attempted at controlled distances; time to first valid shot; recovery between balls; successful shots per shooting opportunity; healthy-device uptime; voltage under repeatable combined load; driver intervention count. Simulations and replay can catch command/estimator regressions but cannot certify physical scoring or prove that software remedies a wiring fault.

My recommendation is a reliability-focused rewrite of the **coordination layer**, retaining the existing subsystem IO and hardware. Its potential is supported by the timing, power and source findings above; an extra-points estimate is not supported by the available archive.
