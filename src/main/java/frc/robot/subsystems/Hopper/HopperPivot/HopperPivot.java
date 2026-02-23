package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperPivot extends SubsystemBase {

    private final HopperPivotIO io;
    private HopperPivotIOInputsAutoLogged inputs = new HopperPivotIOInputsAutoLogged();

    private PIDController controller;
    private ArmFeedforward feedforward;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private double goalAngle = 0.0;

    public HopperPivot(HopperPivotIO io) {
        this.io = io; 
        this.controller = new PIDController(5, 0, 0);
        this.feedforward = new ArmFeedforward(0, 0, 0);

        this.controller.setTolerance(HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE);
        this.profile = new TrapezoidProfile(HopperPivotConstants.HOPPER_CONSTRAINTS);

        SmartDashboard.putData(getName(), this);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperPivot", inputs);
    }

    private void runAngle(Angle angle) {
        setpointState = 
            profile.calculate(
                0.02,
                setpointState,
                new TrapezoidProfile.State(angle.in(Degrees), 0.0));
        
        double volts = 
            controller.calculate(inputs.pivotAngle.in(Degrees), setpointState.position)
                + feedforward.calculate(Degrees.of(setpointState.position).in(Radians), 0);
        
        io.runVoltage(Volts.of(volts));
    }

    public Command setPivotAngle(Supplier<Angle> angle) {
        return this.run(() -> {
            goalAngle = angle.get().in(Degrees);
            runAngle(angle.get());})
        .beforeStarting(() -> {
            profile = new TrapezoidProfile(HopperPivotConstants.HOPPER_CONSTRAINTS);
            setpointState = new TrapezoidProfile.State(inputs.pivotAngle.in(Degrees), 0.0);})
        .withName("Hopper/Pivot/AngleCommand");
    }

    public Command setPivotVoltage(Supplier<Voltage> volts) {
        return this.startEnd(
            () -> io.runVoltage(volts.get()), 
            () -> io.stop()).withName("Hopper/Pivot/VoltageCommand");
    }

    @AutoLogOutput(key = "AtGoal")
    public boolean atGoal(){
        return Math.abs(inputs.pivotAngle.in(Degrees) - goalAngle) 
            < HopperPivotConstants.HOPPER_PIVOT_TOLERANCE;
    }
}