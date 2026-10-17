# Electrical dashboard in AdvantageScope

Double-click `Open Electrical Dashboard.command` in the robot-2026 folder. Choose a recorded run; Enter selects Seneca Q11. The launcher prepares an electrical viewing copy and opens it in AdvantageScope. The first generation takes a few seconds. The source logs remain unchanged.

The dashboard is a native AdvantageScope Mechanism2d tab, not a screenshot. It follows the same selected time as the graphs, tables, robot pose and video. AdvantageScope may briefly display a blank diagram while decoding its fields. Selecting a match timestamp also ensures the diagram has initial values to display.

## Views

- **Electrical Map:** color-changing roboRIO, controller battery input, PDH, CAN, radio, Driver Station, 6 V/5 V/3.3 V rails, gyro and three cameras. All24 recorded PDH currents now appear below the logical map. Motor health and physical wiring remain unknown where unsupported; Module1 zero drive feedback during peer motion is an amber observation warning.
- **Electrical Status:** exact explanations behind the colors. Click the timestamp input, paste an FPGA time and press Return for a precise jump. The table contains change times, not uniformly sampled rows.
- **CAN Diagnostics:** utilization as a fraction (0.8 = 80 percent), bus-off count, receive/transmit errors and transmit-full count. These are bus-level metrics, not an identification of the failed CAN connector or device.
- **RIO Rails:** rail voltages and active flags.
- **PDH Faults:** raw active and sticky fault bitmasks alongside total current. A nonzero bitmask is not the number of faults. Sticky bits describe history. Raw PDH masks are now amber pending verification of the actual deployed native decoder; they do not certify a physical breaker failure. See Q11_FAILURE_ANALYSIS.md for source-code concerns and their uncertain applicability.
- **PDH Channels:** the channel-current array, channel count, voltage and temperature. Array position is a PDH channel index; it is not yet a mechanism name.
- The earlier electrical graphs, cameras, tag/pose, robot and video tabs remain available.

If another layout is active, use File → Import Layout and select `analysis/1257-electrical-dashboard.json`. Importing a layout resets the open window; rerun the launcher afterward to load the chosen run. Future launches from a stopped app select this electrical layout automatically. Running launches preserve the current layout.

## Colors and evidence

| Color | Meaning |
| --- | --- |
| Green | No selected warning, or a logged connected/active flag is true. This does not certify the complete component. |
| Amber | Review warning: controller input below 9 V, CAN utilization above 80 percent, a recent error-counter increment, historical PDH sticky faults, or disabled roboRIO outputs outside an enabled interval. |
| Red | Brownout or logged brownout threshold crossed, inactive power rail, disconnected flag, bus-off increment, or DS-enabled/FPGA-inactive disagreement. A disconnected flag before a match need not indicate a failure. |
| Gray | Missing or unsupported evidence. Physical wiring links deliberately remain gray. |

The 9 V and 80 percent cutoffs are review choices, not hardware failure specifications. Counter increments highlight for one second; an old nonzero counter does not stay highlighted. Counter resets are not new faults. Change-only telemetry holds the most recent value; a constant signal is not automatically stale. Video has about ±1 second of alignment uncertainty. Source timestamps are retained; different devices sample at different rates.

The BATTERY node uses `/SystemStats/BatteryVoltage`, measured at the controller input. It does not establish voltage at the battery terminals or prove a bad battery. PDH voltage is a separate measurement. The RIO node highlights the brownout/output-enable condition; it does not claim the processor rebooted. The radio flag does not prove a radio reboot.

## Q11 walkthrough

Use the original FPGA times, not the previous focus excerpt's zero-based times:

1. Select **410** in Electrical Status and return to Electrical Map for the lead-in.
2. Select **417.593364** to inspect the voltage minimum. The controller input, RIO and 6 V rail are red, while the radio and DS flags still report connected. The PDH independently reports a nonzero active fault mask.
3. Select **418.2** to see the camera-connected flags after they fall; the controller has already disabled outputs.
4. Select **420** to see the radio disconnected flag.
5. Select **423** to see Driver Station detachment too.

Click Play or scrub the native timeline to watch the colors update. Electrical Status explains a highlighted component; the electrical graphs show its original measurements. To synchronize full Q11 video, use `videos/durFirbjol0.mp4` and the saved offset in `analysis/video-sync/manifest.json`: original log time = full video time + 384.057 s. The focused 15-second video from the earlier exercise has a different origin and must not be locked as if it were the full match video.

## Wiring and individual device coverage

This is a logical health diagram. The current evidence does not provide physical CAN daisy-chain order, connector locations, termination measurements, verified PDH channel-to-mechanism assignments or per-controller health flags in Q11. It therefore cannot highlight a specific failed wire, certify motor-controller connectivity from current alone, or reconstruct the actual wiring harness. A team wiring diagram/channel map and logged per-device communication/fault flags would allow that extension. Code CAN IDs alone do not establish physical order.

The converter works on original season WPILOGs in `logs/2026`. Unavailable signals remain gray. It does not change or deploy robot software, and this version is for recorded-log replay rather than live diagnostic generation on the robot.

## Reproduction and verification

From the project root:

```sh
robot-2026/.venv/bin/python robot-2026/scripts/electrical_dashboard.py akit_26-03-14_17-33-19_njtab_q11.wpilog
robot-2026/.venv/bin/python -m unittest discover -s robot-2026/scripts -p 'test_*.py'
```

Generated logs live under ignored `logs/electrical/`, outside the season analysis inventory. Receipts under `analysis/electrical/` record source/output hashes, the converter revision, color-change events and preservation checks. Every original non-padding control/data record retains its entry ID, timestamp, payload and order; WPILOG record headers are re-encoded because original files can have trailing block padding. Generated schematic data uses `/Electrical/` only.

The native rendering contract is pinned to AdvantageScope v26.0.2 sources under `provenance/electrical-dashboard/`. See [AdvantageScope Mechanism documentation](https://docs.advantagescope.org/tab-reference/mechanism/) and the saved source receipts. Native rendering was checked on Q11 at 417.593364 s; rule tests cover missing values, exact event boundaries, counter expiry/reset, sticky versus active faults and disabled operation.


## Extended power, motor and motion views

Electrical Map includes24 raw channel-current bars numbered00–23. Full bar length is60A; exact values are not clipped and appear in **PDH Amps0–7**, **8–15** and **16–23**. Blue shows recorded current, not healthy wiring. Channels3–5 and18–23 are amber for review of the declared logger source, not proof of corruption or a tripped breaker. Physical channel-to-device assignments remain unknown.

**Motor Map** shows16 groups. M0D/M0T mean Module0 drive/turn, through M3D/M3T. Blue bars use80A full scale and purple bars12V; gray means no available metric. **Motor Values**, **Motor Amps**, **Motor Volts**, **Motor Speed rad-s** and **Motor Temp C** retain the available measurements. Controller output current and PDH input current are different measurements and must not be equated to identify wiring.

**Swerve Measured vs Setpoint** overlays blue measured and orange optimized wheel states, with gyro heading. Module order FL/FR/BL/BR and the4.6m/s,0.6731m display settings come from the recorded base code. The log marks that working tree dirty. At415.993482s, the front-right/Module1 optimized speed command is4.591m/s and measured speed0. This view exposes a command/feedback mismatch; it is not independent proof the physical wheel did not move.

**Mechanisms2D - Encoder Angles** shows hood and left/right hopper encoder angles with goal overlays. Positions and link lengths are schematic; encoder zeros are not verified physical reference angles. **Robot Alerts** and **Driver Inputs** expose available logged errors/warnings and joystick axes/buttons.

For the paired replay, use `logs/focus/Q11-electrical-focus.wpilog` (original410–425s, no rebasing) and the15-second `.mov` clip. At selected410.000 with frame1, click the Video lock, then create a new video popout from the current tab. Tile the hub left and video right. Main and popout now share the cursor; the earlier stale satellite was replaced. Use the global playback button in the hub to play all views together. Right-click that button for playback speed and loop settings. Full detail and ranked causes are in **Q11_FAILURE_ANALYSIS.md**.

Motor current/voltage bar lengths show absolute magnitudes. Signed voltage and speed values remain in the native plots and tables. To reopen the focused incident, double-click **Q11 Brownout Replay.command**.
