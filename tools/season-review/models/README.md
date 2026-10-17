# Team 1257 Nautilus — AdvantageScope model (approximate)

This is a *functional approximate visualization*, not an exported team CAD model. The 26.5-inch swerve wheelbase/track and 35-inch bumper-inclusive robot width were derived from `DriveConstants.java` in `FRC1257/2026-Robot` master (retrieved October 2026). Shooter/hopper/intake physical geometry is illustrative and must be calibrated against CAD.

## Install

In AdvantageScope (macOS): `AdvantageScope > Show Assets Folder`. Copy the entire `Robot_1257_Nautilus_2026` directory there. Restart AdvantageScope if the model is not immediately listed. Alternatively select the parent directory of this asset using `Use Custom Assets Folder`.

Open a log or connect to the simulator; create a 3D Field tab and drag the drivetrain's `Pose2d`/`Pose3d` output into **Poses**. Choose `1257 Nautilus 2026 (approximate)` as the robot model. Add the logged `Mechanism3d/Components` pose array as child component poses. Its order is intake, hood, as seen in `NautilusMechanism3d.java`.

If the mechanism appears misaligned, modify component offsets and re-export from a measured robot or CAD model. In particular, the existing code's hood pose uses origin (0,0,0) rather than a measured pivot translation, so the hood visualization is illustrative until the robot code/model coordinate origins are calibrated.

## Local simulation

- Install WPILib 2026 and its VS Code/JDK17 tools.
- `git clone https://github.com/FRC1257/2026-Robot.git`
- `cd 2026-Robot && chmod +x gradlew && ./gradlew build && ./gradlew simulateJava`
- In AdvantageScope: connect to simulator (NT4). SIM mode is already selected in Constants.java and writes sim_logs/.

## Verified sources

- https://github.com/FRC1257/2026-Robot/blob/master/src/main/java/frc/robot/subsystems/drive/DriveConstants.java
- https://github.com/FRC1257/2026-Robot/blob/master/src/main/java/frc/robot/NautilusMechanism3d.java
- https://docs.advantagescope.org/more-features/custom-assets/
