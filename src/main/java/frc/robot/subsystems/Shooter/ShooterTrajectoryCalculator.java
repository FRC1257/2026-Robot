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

        // do the virtual robot position loop here

        Angle hoodAngle = hoodAngleMap.get(robotToTargetDistance);




        return latestParameters;
    }

    public void resetParameters() {
        latestParameters = null;
    }


}
