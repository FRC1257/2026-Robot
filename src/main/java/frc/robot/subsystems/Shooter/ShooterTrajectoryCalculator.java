package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

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
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));
        hoodAngleMap.put(Meters.of(0.0), Radians.of(0));

        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));
        flywheelSpeedMap.put(Meters.of(0.0), RadiansPerSecond.of(0.0));

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
    

    public void configureSuppliers(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier, Supplier<ChassisSpeeds> fieldRelativeVelocitySupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.fieldRelativeVelocitySupplier = fieldRelativeVelocitySupplier;
    }

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

        for(int i = 0; i<20; i++) {
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


}
