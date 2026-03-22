package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.networktables.NetworkTable; 


/*
 * We are importing NT4 classes
 * DoubleArraySubscriber = gets the botpose arrays from the Limelight
 * DoubleArrayPublisher = used to the tell the Limelight the robot's gyro heading
 * TimesstampedDoubleArray = contains the data plus the timestamp of when the data was received by the RIO (essential for latency calculations)
 * what is a subsriber? - listener; what is a publisher? - speaker
 * Why do we use them? - much easier to connect quickly with Limelight; less time than getters and setters
 */
public class VisionIOLimelight implements VisionIO {

    private final String name;
    private final DoubleArraySubscriber mt1Sub;
    private final DoubleArraySubscriber mt2Sub;
    private final DoubleArrayPublisher orientationPub;
  
    // For Autoalign (tx, ty, tv)
    private final DoubleSubscriber txSub;
    private final DoubleSubscriber tySub;
    private final DoubleSubscriber tvSub;

    double lastTimestamp = 0;

    public VisionIOLimelight(String name) {
        this.name = name;
        // table for each specific limelight (model is designed to have 2 atm)
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        
        // subscribe (aka listen to) megatag 1 and megatag 2 pose arrays
        // publish (aka speak) orientation to the limelight (from gyro)
        this.mt1Sub = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
        this.mt2Sub = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
        this.orientationPub = table.getDoubleArrayTopic("orientation").publish();
        
        /*
         * tx = horizontal offset (negative means left) (positive means right)
         * ty = vertical offset
         * tv = whether the limelight has a valid target or not (0 or 1) 
         */
        this.txSub = table.getDoubleTopic("tx").subscribe(0.0);
        this.tySub = table.getDoubleTopic("ty").subscribe(0.0);
        this.tvSub = table.getDoubleTopic("tv").subscribe(0.0);

        // Diagnostic: record which keys the Limelight table currently exposes so we can validate NT layout
        try {
            Logger.recordOutput("Vision/Limelight/" + name + "/has_botpose_wpiblue", table.containsKey("botpose_wpiblue") ? 1 : 0);
            Logger.recordOutput("Vision/Limelight/" + name + "/has_botpose_orb_wpiblue", table.containsKey("botpose_orb_wpiblue") ? 1 : 0);
            Logger.recordOutput("Vision/Limelight/" + name + "/has_orientation", table.containsKey("orientation") ? 1 : 0);
            Logger.recordOutput("Vision/Limelight/" + name + "/has_tx", table.containsKey("tx") ? 1 : 0);
            Logger.recordOutput("Vision/Limelight/" + name + "/has_tv", table.containsKey("tv") ? 1 : 0);
        } catch (Exception e) {
            // best-effort logging
        }
    }

    private double lastSeenTimestamp = 0.0;

    @Override
    public void updateInputs(VisionIOInputs inputs, Rotation2d gyroHeading) {
        /*
         * MegaTag 2 requires that we also give the limelight our robot's gyro heading atm
         * helps remove disambiguity in pose estimation
         * This makes MegaTag 2 a little better than Mega Tag 1
         *  While it works great for multitag, megatag 1 is subject to pose flipping
         *  Because Megatag 2 uses the gyro heading, we can prevent that issue
         */
        orientationPub.set(new double[] {gyroHeading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

        List<PoseObservation> observations = new ArrayList<>();

        // Process MegaTag 1
        for (var sample : mt1Sub.readQueue()) {
            var obs = parseSample(sample, PoseObservationType.MEGATAG_1);
            if (obs != null) {
                observations.add(obs);
            }
        }

        // Process MegaTag 2
        for (var sample : mt2Sub.readQueue()) {
            var obs = parseSample(sample, PoseObservationType.MEGATAG_2);
            if (obs != null) {
                observations.add(obs);
            }
        }

        inputs.poseObservations = observations.toArray(new PoseObservation[0]);
        
        /*
         * check is there is a valid target and then get tx and ty for autoalign
         */
        inputs.latestTarget = new TargetObservation(tvSub.get() > 0, txSub.get(), tySub.get());

        //if we get new observations, update the last seen timestamp so we can check for connectivity
        if (observations.size() > 0) {
            lastSeenTimestamp = Timer.getFPGATimestamp();
        }
        inputs.connected = (Timer.getFPGATimestamp() - lastSeenTimestamp) < 0.25;

       
    }

    private PoseObservation parseSample(TimestampedDoubleArray sample, PoseObservationType type) {
        double[] data = sample.value;

        // Basic defensive checks
        if (data == null || data.length < 10) {
            return null;
        }

        // data[6] is latency (ms) per our expectation. sample.timestamp is in microseconds.
        double timestamp = (sample.timestamp / 1e6) - (data[6] / 1000.0);

        // Sanity check key values
        double x = data[0];
        double y = data[1];
        double yawDeg = data[5];
        double tagCount = data[7];
        double ambiguity = data[8];
        double avgDist = data[9];

        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(yawDeg) || !Double.isFinite(timestamp)) {
            return null;
        }

        // Drop samples whose timestamp is absurd (e.g., far in future)
        double now = Timer.getFPGATimestamp();
        if (timestamp > now + 0.5 || timestamp < now - 10.0) {
            return null;
        }

        // Record last seen for freshness checks
        lastSeenTimestamp = Math.max(lastSeenTimestamp, timestamp);

        // Debug/log raw sample and computed timestamp for tuning (records continuously but can be filtered later)
        try {
            Logger.recordOutput("Vision/Limelight/LastRawSample", data);
            Logger.recordOutput("Vision/Limelight/LastSampleTimestamp", timestamp);
        } catch (Exception e) {
            // Logging shouldn't crash vision loop
        }

        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yawDeg));

        return new PoseObservation(
            timestamp,
            pose,
            (int) tagCount, // number of tags that were observed
            ambiguity, // Ambiguity/Span
            avgDist, // how far away the tags are
            type);
    }
}

  
   