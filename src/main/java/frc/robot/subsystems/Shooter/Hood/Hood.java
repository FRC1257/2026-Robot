package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;

public class Hood extends SubsystemBase {

    private final HoodIO io; 
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private PIDController controller;
    private ArmFeedforward feedforward;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private double goalAngle = 0.0;

    private double hoodOffset = 0.0;
    private boolean isZeroed = false;

    public Hood(HoodIO io) {
        this.io = io; 
        this.controller = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
        this.feedforward = new ArmFeedforward(0, 4, 0);

        this.controller.setTolerance(HoodConstants.HOOD_ANGLE_TOLERANCE);
        this.profile = new TrapezoidProfile(HoodConstants.HOOD_CONSTRAINTS);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }
    
    /**
     * @return the current angle of the hood in radians, accounting for any offset from zeroing
     */

    @AutoLogOutput(key = "Shooter/Hood/MeasuredAngleRads")
    public double getMeasuredAngle() {
        return inputs.hoodAngle.in(Radians) + hoodOffset;
    }

    /**
     * @return true if the hood is at its goal angle and has been zeroed, false otherwise
     */

    @AutoLogOutput(key = "Shooter/Hood/atGoal")
    public boolean isAtGoal() {
        return Math.abs(getMeasuredAngle() - goalAngle) < HoodConstants.HOOD_ANGLE_TOLERANCE && isZeroed;
    }

    /**
     * Sets the current position of the hood as the zero angle. 
     * This should be called when the hood is at its minimum angle, as defined in HoodConstants,
     * to ensure that all future angle commands are accurate.
     */

    public void zero() {
        hoodOffset = HoodConstants.HOOD_MIN_ANGLE.in(Radians) - inputs.hoodAngle.in(Radians);
        isZeroed = true;
    }

    /**
     * Runs the hood at the specified angle using a PID controller and feedforward.
     * @param angle the desired angle to run the hood to, in radians
     */

    private void runAngle(Angle angle) {
        setpointState = 
            profile.calculate(
                0.02,
                setpointState,
                new TrapezoidProfile.State(angle.in(Radians), 0.0));
        
        double volts = 
            controller.calculate(getMeasuredAngle(), setpointState.position)
                + feedforward.calculate(
                    setpointState.position, 
                    setpointState.velocity);
        
        io.runVoltage(Volts.of(volts));
    }

    /**
     * Runs the hood to the specified angle using a PID controller and feedforward, with the angle supplied by a Supplier.
     * @param angle a Supplier that provides the desired angle to run the hood to, in radians
     * @return a Command that runs the hood to the specified angle when executed
     */

    public Command runAngleCommand(Supplier<Angle> angle) {
        return run(() -> {
            goalAngle = angle.get().in(Radians);
            runAngle(angle.get());})
        .beforeStarting(() -> {
            profile = new TrapezoidProfile(HoodConstants.HOOD_CONSTRAINTS);
            setpointState = new TrapezoidProfile.State(getMeasuredAngle(), 0.0);
            })
        .withName("Shooter/Hood/AngleCommand");
    }

    /**
     * Runs the hood at the specified voltage, with the voltage supplied by a Supplier. 
     * @param volts a Supplier that provides the desired voltage to run the hood at
     * @return a Command that runs the hood at the specified voltage when executed
     */

    public Command runVoltageCommand(Supplier<Voltage> volts) {
        return runEnd(
            () -> io.runVoltage(volts.get()),
            () -> io.runVoltage(Volts.of(0.0)))
        .withName("Shooter/Hood/VoltageCommand");
    }

    /**
     * Runs the hood to the angle specified by the {@link ShooterTrajectoryCalculator} using a PID controller and feedforward.
     * @return a Command that runs the hood to the target angle when executed
     */

    public Command runTargetedCommand() {
        return run(()-> {
            goalAngle = ShooterTrajectoryCalculator.getInstance().getParameters().hoodAngle().in(Radians);
            runAngle(ShooterTrajectoryCalculator.getInstance().getParameters().hoodAngle());})
        .beforeStarting(() -> {
            profile = new TrapezoidProfile(HoodConstants.HOOD_CONSTRAINTS);
            setpointState = new TrapezoidProfile.State(getMeasuredAngle(), 0.0);
            })
        .withName("Shooter/Hood/TargetedCommand");
    }

    /**
     * Runs the hood at a constant voltage until it is zeroed, as determined by the hood velocity being below a certain threshold for a certain amount of time.
     * @return a Command that runs the hood at a constant voltage until it is zeroed
     */
    
    public Command zeroCommand() {
        return runVoltageCommand(() -> HoodConstants.HOMING_VOLTAGE)
            .raceWith(Commands.waitSeconds(0.5)
                .andThen(Commands.waitUntil(() -> Math.abs(inputs.hoodVelocity.magnitude()) <= HoodConstants.HOMING_VELOCITY_THRESHOLD)))
            .andThen(this::zero)
            .withTimeout(3.0)
        .withName("Shooter/Hood/ZeroCommand");
    }

}
