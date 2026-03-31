package frc.Paralib.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Units.UnitUtil;

public class SnailMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends SubsystemBase {
    
    protected U motorIO;
    protected T inputs;

    protected TrapezoidProfile profile;
    protected TrapezoidProfile.State goal = new TrapezoidProfile.State();
    protected TrapezoidProfile.State setpoint = null;

    private Distance goalPosition = Meters.of(0.0);
    private Angle goalAngle = Radians.of(0.0);

    public SnailMotorSubsystem(U motorIO, T inputs) {
        this.motorIO = motorIO;
        this.inputs = inputs;
    }

    @Override
    public void periodic() {
        motorIO.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    protected Command setVoltageCommand(Supplier<Voltage> voltage) {
        return runEnd(
            () -> motorIO.setVoltage(voltage.get()),
            () -> motorIO.setVoltage(Volts.of(0.0)));
    }

    protected Command setVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runEnd(
            () -> motorIO.setVelocity(velocity.get()),
            () -> motorIO.setVelocity(RadiansPerSecond.of(0.0)));
    }

    protected Command setAngleCommand(Supplier<Angle> angle) {
        return run(() -> motorIO.setAngle(angle.get()));
    }

    protected Command setPositionCommand(Supplier<Distance> position) {
        return run(() -> motorIO.setPosition(position.get()));
    }

    protected Command setProfiledAngleCommand(Supplier<Angle> angle) {
        return run(() -> {
            Angle angleClamped = UnitUtil.clamp(angle.get(), Radians.of(0), Radians.of(2 * Math.PI));
            goalAngle = angleClamped;
            goal = new TrapezoidProfile.State(angleClamped.in(Radians), 0.0);
            setpoint = profile.calculate(LoggedRobot.defaultPeriodSecs, setpoint, goal);

            Logger.recordOutput(getName() + "/GoalAngle", goalAngle);
            Logger.recordOutput(getName() + "/Setpoint", setpoint.position);

            motorIO.setAngle(Radians.of(setpoint.position));
        }).beforeStarting(() -> {
            Angle angleClamped = UnitUtil.clamp(angle.get(), Radians.of(0), Radians.of(2 * Math.PI));
            goalAngle = angleClamped;
            setpoint = new TrapezoidProfile.State(inputs.positionRadians.in(Radians), inputs.velocityRadsPerSec.in(RadiansPerSecond));
        });
    }

    protected Command setProfiledPositionCommand(Supplier<Distance> position) {
        return run(() -> {
            Distance positionClamped = UnitUtil.clamp(position.get(), Meters.of(0), Meters.of(10));
            goalPosition = positionClamped;
            goal = new TrapezoidProfile.State(positionClamped.in(Meters), 0.0);
            setpoint = profile.calculate(LoggedRobot.defaultPeriodSecs, setpoint, goal);

            Logger.recordOutput(getName() + "/GoalPosition", goalPosition);
            Logger.recordOutput(getName() + "/Setpoint", setpoint.position);

            motorIO.setPosition(Meters.of(setpoint.position));
        }).beforeStarting(() -> {
            Distance positionClamped = UnitUtil.clamp(position.get(), Meters.of(0), Meters.of(10));
            goalPosition = positionClamped;
            setpoint = new TrapezoidProfile.State(inputs.positionMeters.in(Meters), inputs.velocityRadsPerSec.in(RadiansPerSecond));
        });
    }

    public Distance getPosition() {
        return inputs.positionMeters;
    }

    public Angle getAngle() {
        return inputs.positionRadians;
    }
    
    public AngularVelocity getVelocity() {
        return inputs.velocityRadsPerSec;
    }

    public Voltage getVoltage() {
        return inputs.voltageApplied;
    }

    public Current getCurrent() {
        return inputs.currentAmps;
    }

    public Temperature getTemperature() {
        return inputs.temperatureCelsius;
    }

    public Angle getGoalAngle() {
        return goalAngle;
    }

    public Distance getGoalPosition() {
        return goalPosition;
    }

}
