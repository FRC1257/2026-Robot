// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.drive.DriveControls.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.Hopper.HopperIntake.HopperIntake;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeConstants;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeIO;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeIOSparkMax;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeIOSim;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivot;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotIO;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotIOSim;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotIOSparkMax;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final HopperIntake hopperIntake;
  private final HopperPivot hopperPivot;

  private Mechanism2d HopperPivotMechanism = new Mechanism2d(3, 3);


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
        // Real robot, instantiate hardware IO implementations
      case REAL:
        drive =
            new Drive(
                new GyroIOReal(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new VisionIOPhoton());
        hopperIntake 
         =  new HopperIntake(new HopperIntakeIOSparkMax());
        
         hopperPivot = new HopperPivot(new HopperPivotIOSparkMax());
         break;

        // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new VisionIOSim());
        
        hopperIntake = new HopperIntake(new HopperIntakeIOSim());
        hopperPivot = new HopperPivot(new HopperPivotIOSim());
        break;

        // Replayed robot, disable IO implementations
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new VisionIO() {});

        hopperIntake = new HopperIntake(new HopperIntakeIO() {});
        hopperPivot = new HopperPivot(new HopperPivotIO() {});
    }

    // Set up robot state manager

    MechanismRoot2d hopperPivotRoot = HopperPivotMechanism.getRoot("Hopper Pivot", 1.5, 0.5);
    hopperPivotRoot.append(hopperPivot.getPivot());
    SmartDashboard.putData("Hopper Pivot Mechanism", HopperPivotMechanism);

    // Set up auto routines
    /* NamedCommands.registerCommand(
    "Run Flywheel",
    Commands.startEnd(
            () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
        .withTimeout(5.0)); */
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterization, drive::getCharacterizationVelocity));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    configureControls();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));

    hopperPivot.setDefaultCommand(hopperPivot.setPivotVoltage(() -> Volts.of(0.0)));
    hopperIntake.setDefaultCommand(hopperIntake.stopIntake());

    // DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

    DRIVE_STOP.onTrue(
        new InstantCommand(
            () -> {
              drive.stopWithX();
              drive.resetYaw();
            },
            drive));
    
    new Trigger(() -> (int) Timer.getMatchTime() == 20.0).onTrue(getRumbleBoth());

    ANGLE_HOPPER.onTrue(hopperPivot.setPivotAngle(() -> Degrees.of(65.0)));
    HOPPER_PIVOT_VOLTAGE.onTrue(hopperPivot.setPivotVoltage(() -> Volts.of(5.0)));

    HOPPER_INTAKE.onTrue(hopperIntake.runIntake());
    HOPPER_OUTTAKE.onTrue(hopperIntake.runOutake());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return DriveCommands.feedforwardCharacterization(drive);
    // return DriveCommands.wheelRadiusCharacterization(drive);
  }

}
