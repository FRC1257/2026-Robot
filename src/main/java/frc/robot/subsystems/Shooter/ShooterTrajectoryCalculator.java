package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.drive.AllianceFlipUtil;
import frc.robot.util.misc.GeometryUtilities;
import frc.robot.util.misc.LoggedTunableMeasure;
import frc.robot.util.misc.LoggedTunableNumber;
import frc.robot.util.misc.UnitInterpolation;

public class ShooterTrajectoryCalculator {

    private static ShooterTrajectoryCalculator instance; 

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<ChassisSpeeds> robotVelocitySupplier;
    private Supplier<ChassisSpeeds> fieldRelativeVelocitySupplier;

    public static ShooterTrajectoryCalculator getInstance() {
        if (instance == null) instance = new ShooterTrajectoryCalculator();
        return instance;
    }


    public record ShooterTrajectoryParameters(
        boolean isValid, 
        Rotation2d driveAngle,
        double driveVelocity,
        AngularVelocity flywheelVelocity,
        Angle hoodAngle,
        boolean passing
    ) {}

    private ShooterTrajectoryParameters latestParameters = null; 

    private static final InterpolatingTreeMap<Distance, Angle> hoodAngleMap =
        new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(Radians));
    private static final InterpolatingTreeMap<Distance, AngularVelocity> flywheelSpeedMap =
        new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(RadiansPerSecond));
    private static final InterpolatingTreeMap<Distance, Time> timeOfFlightMap =
        new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(Seconds));

    static {
        hoodAngleMap.put(Meters.of(0.96 + 0.5969), Radians.of(0.0));
        hoodAngleMap.put(Meters.of(1.16 + 0.5969), Radians.of(0.025));
        hoodAngleMap.put(Meters.of(1.58 + 0.5969), Radians.of(0.032));
        hoodAngleMap.put(Meters.of(2.07 + 0.5969), Radians.of(0.039));
        hoodAngleMap.put(Meters.of(2.37 + 0.5969), Radians.of(0.048));
        hoodAngleMap.put(Meters.of(2.47 + 0.5969), Radians.of(0.053));
        hoodAngleMap.put(Meters.of(2.70 + 0.5969), Radians.of(0.061));
        hoodAngleMap.put(Meters.of(2.94 + 0.5969), Radians.of(0.065));
        hoodAngleMap.put(Meters.of(3.48 + 0.5969), Radians.of(0.065));
        hoodAngleMap.put(Meters.of(3.92 + 0.5969), Radians.of(0.15));
        hoodAngleMap.put(Meters.of(4.35 + 0.5969), Radians.of(0.158));

        flywheelSpeedMap.put(Meters.of(0.96 + 0.5969), RadiansPerSecond.of(300));
        flywheelSpeedMap.put(Meters.of(1.16 + 0.5969), RadiansPerSecond.of(300));
        flywheelSpeedMap.put(Meters.of(1.58 + 0.5969), RadiansPerSecond.of(315));
        flywheelSpeedMap.put(Meters.of(2.07 + 0.5969), RadiansPerSecond.of(330));
        flywheelSpeedMap.put(Meters.of(2.37 + 0.5969), RadiansPerSecond.of(340));
        flywheelSpeedMap.put(Meters.of(2.47 + 0.5969), RadiansPerSecond.of(350));
        flywheelSpeedMap.put(Meters.of(2.70 + 0.5969), RadiansPerSecond.of(355));
        flywheelSpeedMap.put(Meters.of(2.94 + 0.5969), RadiansPerSecond.of(360));
        flywheelSpeedMap.put(Meters.of(3.48 + 0.5969), RadiansPerSecond.of(362));
        flywheelSpeedMap.put(Meters.of(3.92 + 0.5969), RadiansPerSecond.of(370));
        flywheelSpeedMap.put(Meters.of(4.35 + 0.5969), RadiansPerSecond.of(390));
        
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
        timeOfFlightMap.put(Meters.of(0.0), Seconds.of(0.0));
    }

    private static final InterpolatingTreeMap<Distance, Angle> passingHoodAngleMap =
        new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(Radians));
    private static final InterpolatingTreeMap<Distance, AngularVelocity> passingFlywheelSpeedMap =
        new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(RadiansPerSecond));
    private static final InterpolatingTreeMap<Distance, Time> passingTimeOfFlightMap =
        new InterpolatingTreeMap<>(UnitInterpolation.inverseInterpolate(), UnitInterpolation.Interpolator(Seconds));
    
    private static final LoggedTunableNumber phaseDelay = new LoggedTunableNumber("/ShooterTrajectoryCalculator/phaseDelay", 0.02);
    
    public ShooterTrajectoryParameters getParameters() {
        if(latestParameters != null) return latestParameters;

        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds robotRelativeVelocity = robotVelocitySupplier.get();

        robotPose = robotPose.exp(new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond*phaseDelay.get(),
            robotRelativeVelocity.vyMetersPerSecond*phaseDelay.get(),
            robotRelativeVelocity.omegaRadiansPerSecond*phaseDelay.get()
        ));
        
        Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Distance robotToTargetDistance = Meters.of(target.getDistance(robotPose.getTranslation()));

        ChassisSpeeds fieldRelativeVelocity = fieldRelativeVelocitySupplier.get();
        Rotation2d robotAngle = robotPose.getRotation();
        ChassisSpeeds shooterVelocity = GeometryUtilities.transformVelocity(fieldRelativeVelocity, robotPose.getTranslation(), robotAngle);

        Time timeOfFlight = timeOfFlightMap.get(robotToTargetDistance);
        Pose2d lookaheadPose = robotPose;
        Distance lookaheadTargetDistance = robotToTargetDistance;

        for(int i = 0; i<10; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadTargetDistance);
            lookaheadPose = 
                new Pose2d(
                    robotPose.getTranslation().plus(
                        new Translation2d(
                            shooterVelocity.vxMetersPerSecond*timeOfFlight.in(Seconds),
                            shooterVelocity.vyMetersPerSecond*timeOfFlight.in(Seconds)
                        )),
                        robotPose.getRotation()
                    );
            lookaheadTargetDistance = Meters.of(target.getDistance(lookaheadPose.getTranslation()));     
        }

        Angle hoodAngle = hoodAngleMap.get(lookaheadTargetDistance);
        AngularVelocity flywheeVelocity = flywheelSpeedMap.get(lookaheadTargetDistance);

        latestParameters = new ShooterTrajectoryParameters(true, robotAngle, 0, flywheeVelocity, hoodAngle, false);

        return latestParameters;
    }

    public void resetParameters() {
        latestParameters = null;
    }

    public ShooterTrajectoryParameters getStaticParameters(Supplier<Pose2d> robotPose) {
        Translation2d target = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Distance robotToTargetDistance = Meters.of(target.getDistance(robotPose.get().getTranslation()));
        Logger.recordOutput("ShooterTrajectoryCalculator", robotToTargetDistance);
        return new ShooterTrajectoryParameters(true, robotPose.get().getRotation(), 0, flywheelSpeedMap.get(robotToTargetDistance), hoodAngleMap.get(robotToTargetDistance), false);
    }


}
