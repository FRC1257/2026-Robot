package frc.robot.subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Shooter.Hood.HoodConstants.HOOD_MAX_ANGLE;
import static frc.robot.subsystems.Shooter.Hood.HoodConstants.HOOD_MIN_ANGLE;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.NautilusMechanism3d;
import frc.robot.Robot;
import frc.robot.subsystems.Hopper.HopperPivot.HopperPivotConstants;
import frc.robot.subsystems.Shooter.ShooterTrajectoryCalculator;
import frc.robot.util.Units.UnitUtil;
import frc.robot.util.misc.LoggedTunableNumber;

public class Hood extends SubsystemBase {

    private static final LoggedTunableNumber Kp = new LoggedTunableNumber("Hood/Kp", HoodConstants.HOOD_KP);
    private static final LoggedTunableNumber Ki = new LoggedTunableNumber("Hood/Ki", HoodConstants.HOOD_KI);
    private static final LoggedTunableNumber Kd = new LoggedTunableNumber("Hood/Kd", HoodConstants.HOOD_KD);

    private static final LoggedTunableNumber Ks = new LoggedTunableNumber("Hood/Ks", HoodConstants.HOOD_KS);
    private static final LoggedTunableNumber Kg = new LoggedTunableNumber("Hood/Kg", HoodConstants.HOOD_KG);
    private static final LoggedTunableNumber Kv = new LoggedTunableNumber("Hood/Kv", HoodConstants.HOOD_KV);

    private static final LoggedTunableNumber maxVel = new LoggedTunableNumber("Hood/MaxVelocity", HoodConstants.HOOD_CONSTRAINTS.maxVelocity);
    private static final LoggedTunableNumber maxAccel = new LoggedTunableNumber("Hood/MaxAcceleration", HoodConstants.HOOD_CONSTRAINTS.maxAcceleration);


    private static final LoggedTunableNumber hubAngle = new LoggedTunableNumber("Hood/hubAngle", 0.0);

    private final HoodIO io; 
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final Debouncer connectedDebouncer = 
        new Debouncer(0.5, DebounceType.kFalling);

    private final Alert disconnected;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = null;


    private Angle goalAngle = Radians.of(0.0);
    private Angle hoodOffset = Radians.of(0.0);


    public Hood(HoodIO io) {
        this.io = io; 

        disconnected = new Alert("HOOD MOTOR DISCONNECTED", AlertType.kError);

        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get()));    
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if(Kp.hasChanged(hashCode()) || Ki.hasChanged(hashCode()) || Kd.hasChanged(hashCode())) {
            io.setPID(Kp.get(), Ki.get(), Kd.get());
        }

        if(Ks.hasChanged(hashCode()) || Kg.hasChanged(hashCode()) || Kv.hasChanged(hashCode())) {
            io.setFF(Ks.get(), Kv.get(), Kg.get());
        }

         if(maxVel.hasChanged(hashCode()) || maxAccel.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get()));
        }

        NautilusMechanism3d.getMeasured().setHoodAngle(new Rotation2d(getMeasuredAngle()));

        disconnected.set(!connectedDebouncer.calculate(inputs.hoodConnected));

        Robot.batteryLogger.reportCurrentUsage(
            "Hood",
            inputs.hoodCurrentDraw.in(Amps)
        );

    }
    
    /**
     * @return the current angle of the hood in radians, accounting for any offset from zeroing
     */

    @AutoLogOutput(key = "Shooter/Hood/MeasuredAngleRads")
    public Angle getMeasuredAngle() {
        return inputs.hoodAngle.plus(hoodOffset);
    }

    /**
     * @return true if the hood is at its goal angle and has been zeroed, false otherwise
     */

    @AutoLogOutput(key = "Shooter/Hood/atGoal")
    public Trigger isAtGoal() {
        return new Trigger(() -> getMeasuredAngle().minus(goalAngle).abs(Radians) < HoodConstants.HOOD_ANGLE_TOLERANCE);
    }

    /**
     * Runs the hood to the specified angle using a PID controller and feedforward, with the angle supplied by a Supplier.
     * @param angle a Supplier that provides the desired angle to run the hood to, in radians
     * @return a Command that runs the hood to the specified angle when executed
     */

    public Command runAngleCommand(Supplier<Angle> angle) {
        return run(() -> {
            Angle clamped = UnitUtil.clamp(angle.get(), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
            goalAngle = clamped;
            goal = new TrapezoidProfile.State(clamped.in(Radians),0);
            setpoint = profile.calculate(LoggedRobot.defaultPeriodSecs, setpoint, goal);

            Logger.recordOutput("Hood/setpoint", setpoint.position);
            Logger.recordOutput("Hood/goal", goal.position);

            io.runAngle(setpoint.position, setpoint.velocity);
        }).beforeStarting(() -> {
            Angle clamped = UnitUtil.clamp(angle.get(), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
            goalAngle = clamped;
            setpoint = new TrapezoidProfile.State(inputs.hoodAngle.in(Radians), inputs.hoodVelocity.in(RadiansPerSecond));
        });
    }

    public Command runHubAngle() {
        return runAngleCommand(() -> Radians.of(hubAngle.get()));
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

    public Command runTargetedCommand(Supplier<Pose2d> robotPose) {
        return runAngleCommand(() -> ShooterTrajectoryCalculator.getInstance().getStaticParameters(robotPose).hoodAngle());
    }

}
