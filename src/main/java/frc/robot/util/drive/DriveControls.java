package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivers;
import frc.robot.Constants.Operators;
import frc.robot.util.drive.CommandSnailController.DPad;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveControls {
  // Controllers
  public static final CommandSnailController driver = new CommandSnailController(0);
  public static final CommandSnailController operator = new CommandSnailController(1);

  // Useful for things that don't need to be triggered
  private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);
  private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;

  // Drive controls
  public static DoubleSupplier DRIVE_FORWARD = EMPTY_DOUBLE_SUPPLIER;
  public static DoubleSupplier DRIVE_STRAFE = EMPTY_DOUBLE_SUPPLIER;
  public static DoubleSupplier DRIVE_ROTATE = EMPTY_DOUBLE_SUPPLIER;
  public static Trigger DRIVE_SLOW = EMPTY_TRIGGER;
  public static Trigger DRIVE_STOP = EMPTY_TRIGGER;
  public static Trigger DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;

  // Drive Turns
  public static Trigger TURN_90 = EMPTY_TRIGGER;
  public static Trigger TURN_180 = EMPTY_TRIGGER;

  // Rumble Controls
  public static Trigger TIMED_RUMBLE = EMPTY_TRIGGER;
  public static Trigger INTAKE_RUMBLE = EMPTY_TRIGGER;

  // Creates Elastic dropdown menu for Drivers
  public static class DriverChooser {

    private LoggedDashboardChooser<Drivers> driverChooser;

    public DriverChooser() {
      driverChooser = new LoggedDashboardChooser<>("Driver Selection");
      driverChooser.addDefaultOption("Maddie", Drivers.MADDIE);
      driverChooser.addOption("Michael", Drivers.MICHAEL);
      driverChooser.addOption("Gabe", Drivers.GABE);
      driverChooser.addOption("Programmers", Drivers.PROGRAMMERS);
    }

    public Drivers getDriver() {
      Drivers driver = driverChooser.get();
      if (driver == null) {
        return Drivers.MADDIE;
      }
      return driver;
    }
  }

  // Creates Elastic dropdown menu for Operators
  public static class OperatorChooser {

    private LoggedDashboardChooser<Operators> operatorChooser;

    public OperatorChooser() {
      operatorChooser = new LoggedDashboardChooser<>("Operator Selection");
      operatorChooser.addDefaultOption("Kevin", Operators.KEVIN);
      operatorChooser.addOption("Arboria", Operators.ARBORIA);
      operatorChooser.addOption("Antonios", Operators.ANTONIOS);
      operatorChooser.addOption("Programmers", Operators.PROGRAMMERS);
    }

    public Operators getOperator() {
      Operators operator = operatorChooser.get();
      if (operator == null) {
        return Operators.KEVIN;
      }
      return operator;
    }
  }

  // Instantiate the choosers
  private static final DriverChooser driverChooser = new DriverChooser();
  private static final OperatorChooser operatorChooser = new OperatorChooser();

  // Setup the controls
  public static void configureControls() {
    switch (driverChooser.getDriver()) {
      case MADDIE:
      case MICHAEL:
        DRIVE_FORWARD = () -> -driver.getLeftY();
        DRIVE_STRAFE = () -> -driver.getLeftX();
        DRIVE_ROTATE = () -> -driver.getRightX();
        DRIVE_STOP = driver.x();
        DRIVE_SLOW = driver.rightTrigger();
        DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;

        // TURN_90 = driver.y();
        TURN_180 = driver.start();
        break;
      case GABE:
      case PROGRAMMERS:
      default:
        DRIVE_FORWARD = () -> -driver.getLeftY();
        DRIVE_STRAFE = () -> -driver.getLeftX();
        DRIVE_ROTATE = () -> -driver.getRightX();
        DRIVE_STOP = driver.x();
        DRIVE_SLOW = driver.rightBumper();
        DRIVE_ROBOT_RELATIVE = EMPTY_TRIGGER;

        // TURN_90 = driver.y();
        TURN_180 = driver.start();
        break;
    }

    switch (operatorChooser.getOperator()) {
      case ARBORIA:
        break;
      case KEVIN:
        break;

      case ANTONIOS:
        break;
      case PROGRAMMERS:
      default:
        // Operator controls
        break;
    }
  }

  public static void updateDriverAndOperator() {
    driverChooser.getDriver();
    operatorChooser.getOperator();
    configureControls();
  }

  private static Command getRumbleCommand(CommandSnailController driver) {
    return new InstantCommand(() -> driver.rumble(1))
        .andThen(new WaitCommand(1))
        .andThen(() -> driver.rumble(0));
  }

  public static Command getRumbleBoth() {
    return getRumbleCommand(driver).alongWith(getRumbleCommand(operator));
  }
}
