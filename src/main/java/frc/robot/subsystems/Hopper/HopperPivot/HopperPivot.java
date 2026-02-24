package frc.robot.subsystems.Hopper.HopperPivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class HopperPivot extends SubsystemBase {

    private final HopperPivotIO io;
    private HopperPivotIOInputsAutoLogged inputs = new HopperPivotIOInputsAutoLogged();

    private PIDController controller;
    private ArmFeedforward feedforward;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private MechanismLigament2d pivot = new MechanismLigament2d("Hopper Pivot", 0.4, 0, 5, new Color8Bit(Color.kAqua));

    private double goalAngle = 0.0;

    @AutoLogOutput
    public Trigger atGoalTrigger = new Trigger(this::atGoal);

    public HopperPivot(HopperPivotIO io) {
        this.io = io; 
        this.controller = new PIDController(8, 0, 0);
        this.feedforward = new ArmFeedforward(0, 4, 0);

        this.controller.setTolerance(HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE);
        this.profile = new TrapezoidProfile(HopperPivotConstants.HOPPER_CONSTRAINTS);

        SmartDashboard.putData(getName(), this);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperPivot", inputs);

        pivot.setAngle(inputs.pivotAngle.in(Degrees));
    }

    private void runAngle(Angle angle) {
        setpointState = 
            profile.calculate(
                0.02,
                setpointState,
                new TrapezoidProfile.State(angle.in(Radians), 0.0));
        
        double volts = 
            controller.calculate(inputs.pivotAngle.in(Radians), setpointState.position)
                + feedforward.calculate(
                    setpointState.position, 
                    setpointState.velocity);
        
        io.runVoltage(Volts.of(volts));
    }

    public Command setPivotAngle(Supplier<Angle> angle) {
        return this.run(() -> {
            runAngle(angle.get());})
        .beforeStarting(() -> {
            goalAngle = angle.get().in(Radians);
            profile = new TrapezoidProfile(HopperPivotConstants.HOPPER_CONSTRAINTS);
            setpointState = new TrapezoidProfile.State(inputs.pivotAngle.in(Radians), 0.0);
            })
        .withName("Hopper/Pivot/AngleCommand");
    }

    public Command setPivotVoltage(Supplier<Voltage> volts) {
        return this.startEnd(
            () -> io.runVoltage(volts.get()), 
            () -> io.stop()).withName("Hopper/Pivot/VoltageCommand");
    }

    private boolean atGoal(){
        return Math.abs(inputs.pivotAngle.in(Radians) - goalAngle) 
            < HopperPivotConstants.HOPPER_PIVOT_PID_TOLERANCE;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return pivot.append(mechanism);
    }

    public MechanismLigament2d getPivot() {
        return pivot;
    }
}