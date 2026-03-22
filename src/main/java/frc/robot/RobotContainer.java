
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.drive.DriveControls.*;

import java.io.Flushable;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPose;
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
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.autonomous.AutoChooser;
import frc.robot.util.drive.CommandSnailController;
import frc.robot.util.drive.CommandSnailController.DPad;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.ActiveFloor.ActiveFloor;
import frc.robot.subsystems.ActiveFloor.ActiveFloorIO;
import frc.robot.subsystems.ActiveFloor.ActiveFloorIOSparkMax;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntake;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeConstants;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeIO;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeIOSparkMax;
import frc.robot.subsystems.Hopper.HopperIntake.HopperIntakeIOSim;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivot;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotIO;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotIOSim;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotIOSparkMax;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Kicker.KickerIO;
import frc.robot.subsystems.Kicker.KickerIOSparkMax;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.Shooter.Hood.Hood;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOSparkMax;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final HopperIntake hopperIntake;
  private final HopperPivot hopperPivot;
  private final Kicker kicker;
  private final Hood hood;
  private final ActiveFloor activeFloor;
  private final Flywheel flywheel;

  public static final CommandSnailController driver = new CommandSnailController(0);
  public static final CommandSnailController operator = new CommandSnailController(1);


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoChooser customAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //list of all the various vision IO implementations



    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        //LL
        VisionIOLimelight hopperCam = new VisionIOLimelight(VisionConstants.camNames[0]);
        VisionIOLimelight swerveCam = new VisionIOLimelight(VisionConstants.camNames[1]);

        //photonVision = new VisionIOPhoton();
    
        drive =
            new Drive(
                new GyroIOReal(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
          this.vision = new Vision(
            drive::addVisionMeasurement, // method ref matches VisionConsumer(Pose2d,double,Matrix)
            drive::getRotation,
            hopperCam,
            swerveCam);
 
        hopperIntake = new HopperIntake(new HopperIntakeIOSparkMax());

        hopperPivot = new HopperPivot(new HopperPivotIOSparkMax());
        kicker = new Kicker(new KickerIOSparkMax() {});
        activeFloor = new ActiveFloor(new ActiveFloorIOSparkMax());
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        hood = new Hood(new HoodIOSparkMax());

        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        VisionIOSim simVision = new VisionIOSim();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        
          this.vision = new Vision(
            drive::addVisionMeasurement, // method ref matches VisionConsumer(Pose2d,double,Matrix)
            drive::getRotation,
            simVision);

        hopperIntake = new HopperIntake(new HopperIntakeIOSim());
        hopperPivot = new HopperPivot(new HopperPivotIOSim());
        kicker = new Kicker(new KickerIO() {});
        activeFloor = new ActiveFloor(new ActiveFloorIO() {});
        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
        VisionIO visionIO = new VisionIO() {};
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        
          this.vision = new Vision(
            drive::addVisionMeasurement, // method ref matches VisionConsumer(Pose2d,double,Matrix)
            drive::getRotation,
            visionIO);

        hopperIntake = new HopperIntake(new HopperIntakeIO() {});
        hopperPivot = new HopperPivot(new HopperPivotIO() {});
        kicker = new Kicker(new KickerIO() {});
        activeFloor = new ActiveFloor(new ActiveFloorIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});

        break;
    }





    // Set up robot state manager


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

    customAutoChooser = new AutoChooser(drive, activeFloor, hopperIntake, hopperPivot, kicker, flywheel, hood);
            
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

    hopperPivot.setDefaultCommand(hopperPivot.runIntakeAngle());
    hopperIntake.setDefaultCommand(hopperIntake.stopIntake());
    activeFloor.setDefaultCommand(activeFloor.stopActiveFloor());
    flywheel.setDefaultCommand(flywheel.stopCommand());
    hood.setDefaultCommand(hood.runVoltageCommand(() -> Volts.of(0)));


    driver.x().onTrue(
      new InstantCommand(
        () -> {
          drive.stopWithX();
          drive.resetYaw();
        }
      )
    );


    driver.y()
    .toggleOnTrue(hopperPivot.runStowAngle());

    driver.a()
    .whileTrue(hopperPivot.runAgitate())
    .onFalse(hopperPivot.runIntakeAngle());

    driver.b().whileTrue(drive.lockWheels());

    //FieldConstants.Zones.composedTrench.contains(
      //() -> drive.getPose().getTranslation())
        //.whileTrue(hood.runAngleCommand(() -> Radians.of(0.0)));

    // driver
    //   .rightBumper().whileTrue(
    //     hood.runTargetedCommand()
    //       .alongWith(flywheel.runTargetedCommand())
    //         .alongWith(
    //           Commands.waitUntil(hood::isAtGoal)
    //           .andThen(kicker.runIntake()
    //             .alongWith(activeFloor.runActiveFloor())))
    //   );


    // driver
    //  .leftBumper().whileTrue(
    //    flywheel.runHub().alongWith(hood.runHubAngle())
    //    .alongWith(
    //      kicker.runIntake())
    //      .alongWith(activeFloor.runActiveFloor())
    //      );

  // NEED TO ADD A MANUAL OVERRIDE TO FORCE THE BALLS OUT IF FLYWHEEL ISNT UP TO SPEED

  driver
    .rightBumper().whileTrue(
      flywheel.runTargetedCommand(drive::getPose)
      .alongWith(hood.runTargetedCommand(drive::getPose))
      .alongWith(DriveCommands.joystickHubPoint(drive, DRIVE_FORWARD, DRIVE_STRAFE))
      .alongWith(Commands.waitUntil(flywheel.isAtGoal().and(hood.isAtGoal()))
        .andThen(kicker.runIntake()
        .alongWith(activeFloor.runActiveFloor())
        .alongWith(hopperPivot.runAgitate())
        .alongWith(hopperIntake.runIntake())))
    );

  driver.leftBumper().and(hopperPivot.atIntake()).whileTrue(DriveCommands.alignToTrench(drive));
  driver.leftBumper().and(hopperPivot.atIntake().negate()).whileTrue(hopperPivot.runIntakeAngle().andThen(DriveCommands.alignToTrench(drive)));
  
  // driver
  //   .rightBumper().whileTrue(
  //     kicker.runOuttake()
  //   );

    driver
      .rightTrigger().whileTrue(
        hopperIntake.runIntake()
      );

    driver
      .leftTrigger().whileTrue(
        hopperIntake.runOutake()
      );  
    


  

   new Trigger(() -> Math.abs(operator.getLeftY()) >= 0.1).whileTrue(flywheel.runVoltageCommand(() -> Volts.of(operator.getLeftY())));
  
  

   operator.rightBumper().whileTrue(kicker.runVelocityCommand(() -> RadiansPerSecond.of(-5)));
   operator.leftBumper().whileTrue(flywheel.runVelocityCommand(() -> RadiansPerSecond.of(320)));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return customAutoChooser.getAutoCommand();
    // return DriveCommands.feedforwardCharacterization(drive);
    // return DriveCommands.wheelRadiusCharacterization(drive);
  }

}
