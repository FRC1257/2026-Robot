# FRC Team 1257 — 2026 Robot Code

This repository contains the Java robot code for FRC Team 1257's 2026 competition robot, built for the **Reefscape** game. The codebase is built on top of the [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) logging framework (originally developed by FRC 6328 Mechanical Advantage) and uses WPILib's Command-based architecture.

---

## 🚀 Cool Features

### 1. AdvantageKit Logging & Replay
The robot uses AdvantageKit for comprehensive structured data logging. Every hardware input is captured at each robot loop cycle, enabling:
- **Full match replay**: Re-run any match log through the robot code offline to debug issues
- **Automatic annotation-based logging** via `@AutoLog` and `@AutoLogOutput`
- **Three operation modes** that switch transparently at runtime:
  | Mode | Description |
  |------|-------------|
  | `REAL` | Runs on hardware, logs to USB stick at `/media/sda1/` |
  | `SIM` | Physics simulation, logs to `sim_logs/` |
  | `REPLAY` | Replays a previously recorded log file |

### 2. IO Abstraction Pattern
Every hardware interface is defined as a Java interface (e.g., `ModuleIO`, `GyroIO`, `VisionIO`). Each interface has concrete implementations for real hardware, simulation, and empty stubs for log replay. This means:
- Zero robot-logic code changes are needed to switch between hardware and simulation
- Simulation and replay are first-class citizens, not afterthoughts

### 3. High-Frequency Odometry at 250 Hz
`SparkMaxOdometryThread` runs in a dedicated background thread using a WPILib `Notifier`, sampling wheel encoder and gyro data at **250 Hz** (vs. the standard 50 Hz robot loop). Lock-based synchronization ensures thread safety when the main loop consumes the buffered samples. This results in significantly more accurate pose estimates at high speeds.

### 4. MAXSwerve Drive with Fused Vision Pose Estimation
The drivetrain is a 4-wheel MAXSwerve system with REV SparkMax + NEO motors. The pose estimator (`SwerveDrivePoseEstimator`) fuses high-rate wheel odometry with multi-camera PhotonVision AprilTag detections:
- **Position updates** use multi-tag PnP solved on the coprocessor for accuracy
- **Rotation updates** use the lowest-ambiguity single-tag strategy to combat gyro drift
- Separated covariances allow position and rotation confidence to be tuned independently

### 5. PathPlanner Autonomous with Custom A\* Pathfinding
The robot uses [PathPlanner](https://pathplanner.dev/) for autonomous path following with a custom `LocalADStarAK` adapter that feeds the planner's A\* obstacle-avoidance algorithm through AdvantageKit's logging layer, so pathfinding decisions are fully reproducible in replay.

### 6. Per-Driver Control Profiles
`DriveControls` reads an **Elastic dashboard dropdown** to select a driver profile at runtime. Button bindings and axis assignments are reconfigured on the fly without redeploying code. Supported drivers: Maddie, Michael, Gabe, and Programmers. The same pattern is used for the operator.

### 7. Alliance Coordinate Flipping
`AllianceFlipUtil` is a single utility class that handles all Blue/Red alliance coordinate transforms (poses, rotations, translations). Any code that needs field-relative coordinates calls this utility, ensuring there is never a coordinate-system bug when the alliance color changes.

### 8. Elastic Dashboard Integration
`Elastic.java` wraps the Elastic FRC dashboard API to enable tab selection and JSON-serialized in-match notifications directly from robot code.

### 9. Automatic Build Metadata via `gversion`
The Gradle `gversion` plugin auto-generates `BuildConstants.java` at compile time, embedding the git commit SHA, branch name, build timestamp, and dirty-tree flag. This metadata is logged on startup so every log file is traceable to an exact code state.

### 10. Built-In Motor Characterization (SysId)
Drive and turn motors have built-in quasistatic and dynamic SysId characterization routines exposed as auto-chooser options. Running these on the real robot generates feed-forward constants without any additional tooling.

---

## ⚠️ Potential Issues

### 1. Game Mechanism Subsystems Not Yet Implemented
`RobotContainer` declares `Mechanism2d` placeholders for a coral pivot, elevator, and algae pivot, but none of these have corresponding subsystem classes. The robot currently only has a functional drive and vision system. All game-piece manipulation logic needs to be written before competition.

### 2. `splinePathToPose()` Is Known-Broken
`Drive.java` contains this developer note:
```java
/**
 * This function is flawed because getPose only runs once so the path always
 * starts from the starting pose. Do not use this function until we fix it,
 * use pathfindToPose instead
 */
```
The method is still present and callable. Any caller that uses it will follow a stale path from the robot's position at construction time, not its current position.

### 3. Drive Motor Timeout Warning Left Unresolved
`DriveConstants.java` contains a comment:
```java
// drive motors are timing out for some reason in the logs
```
There is no follow-up resolution. This may indicate a CAN bus communication problem, incorrect CAN IDs, or controller configuration issues that could cause unreliable driving in competition.

### 4. `Constants.useVision` Is Always `true`
```java
public static final boolean useVision = (currentMode == Mode.SIM ? true : true);
```
The conditional here is redundant — vision is always enabled regardless of mode. If vision needs to be disabled (e.g., for testing without cameras or to isolate odometry), this constant provides no control. The flag should at minimum be evaluated against hardware availability, or the dead conditional should be cleaned up.

### 5. Gyro Reset Workaround
`Drive.java` contains this comment about gyro heading resets:
```java
// Yes I know it says that you don't need to reset the gyro rotation,
// but it tweaks out if you don't
```
This suggests an underlying issue with gyro calibration or initialization that was worked around rather than fixed. Gyro instability could affect field-relative driving and vision-odometry fusion.

### 6. Operator Bindings Are Empty Stubs
`DriveControls.configureControls()` contains empty `case` blocks for every operator profile:
```java
case KEVIN:
    break;
case ARBORIA:
    break;
```
No operator controls are mapped. Any game mechanism subsystems added in the future will have no controls until this is completed.

### 7. `DRIVE_SLOW` Binding Is Commented Out
In `RobotContainer.configureButtonBindings()`:
```java
// DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));
```
Slow-mode driving is silently disabled. If a driver expects slow mode to work on the slow-mode trigger, it will not.

### 8. `joystickReefPoint()` Commented Out
A significant auto-rotation command (`joystickReefPoint`) is commented out in `DriveCommands.java`. If this was part of the intended driver workflow for scoring, its absence changes robot behavior without any indication on the dashboard.

### 9. No Automated Tests
`build.gradle` configures JUnit 5, but `src/test/` contains no test classes. Math-heavy utilities like `AllianceFlipUtil` and pose-estimation pipelines have no unit tests. Regressions will only be caught by driving the real robot.

### 10. `TURN_90` Is Commented Out in All Driver Profiles
```java
// TURN_90 = driver.y();
```
The `TURN_90` trigger is declared and initialized to an empty trigger but never bound in any driver profile. Any auto-snap-to-90° functionality is silently unavailable.

---

## 🏗️ Project Structure

```
src/main/java/frc/robot/
├── Robot.java                  # Main robot class
├── RobotContainer.java         # Subsystems, OI, and command wiring
├── Constants.java              # Robot-wide constants and mode selection
├── FieldConstants.java         # Field geometry constants
├── commands/                   # Command implementations
│   ├── DriveCommands.java
│   ├── AlignToPose.java
│   └── FeedForwardCharacterization.java
├── subsystems/
│   ├── drive/                  # Swerve drive subsystem
│   │   ├── Drive.java
│   │   ├── Module.java
│   │   ├── ModuleIO.java / ModuleIOSparkMax.java / ModuleIOSim.java
│   │   ├── GyroIO.java / GyroIOReal.java
│   │   ├── DriveConstants.java
│   │   └── SparkMaxOdometryThread.java
│   └── vision/                 # PhotonVision subsystem
│       ├── VisionIO.java / VisionIOPhoton.java / VisionIOSim.java
│       └── VisionConstants.java
└── util/
    ├── drive/
    │   ├── AllianceFlipUtil.java
    │   ├── DriveControls.java
    │   └── CommandSnailController.java
    ├── autonomous/
    │   └── LocalADStarAK.java
    └── elastic/
        └── Elastic.java
```

---

## 🔧 Building & Deploying

**Build (simulation/CI):**
```bash
./gradlew build
```

**Deploy to robot:**
```bash
./gradlew deploy
```

**Run simulation:**
```bash
./gradlew simulateJava
```

---

## 📦 Key Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| WPILib / GradleRIO | 2026.1.1 | Core robot framework |
| AdvantageKit | 26.0.0 | Logging and replay |
| PathPlannerLib | 2026.1.2 | Autonomous path following |
| REVLib | latest | SparkMax motor controllers |
| PhotonLib | latest | AprilTag vision |
| CTRE Phoenix 6 | latest | CTRE device support |
