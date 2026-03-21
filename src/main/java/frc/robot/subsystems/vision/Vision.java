package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  //interface top send data to drive subsystem
  private final VisionConsumer consumer;
  //current rotation
  private final Supplier<Rotation2d> gyroSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

 //pass in the consumer, gyro supplier, and all the IO implementations (1 for each camera)
  public Vision(VisionConsumer consumer, Supplier<Rotation2d> gyroSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.gyroSupplier = gyroSupplier;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlerts = new Alert[io.length];

    // initialize inputs and alerts for each camera
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] = new Alert("Vision camera " + i + " disconnected", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      // get rotation from gyro, update inputs, and log
      io[i].updateInputs(inputs[i], gyroSupplier.get());
      Logger.processInputs("Vision/Camera" + i, inputs[i]);

      disconnectedAlerts[i].set(!inputs[i].connected);

      // skip the pose estimation if the camera is disconnected
      for (var observation : inputs[i].poseObservations) {
        if (shouldRejectPose(observation)) continue;

        //standard deviation factor based on distance from tag squared / number of tags seen
        //replicated from Metuchen's method of calculation vision measurement noise

        double stdDevFactor = Math.pow(observation.avgTagDist(), 2.0) / observation.tagCount();
        
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        // pushes this stuff to consumer (drive)
        consumer.accept(
            observation.pose(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
        );
      }
    }
  }

  //reject pose observation if too far away, abmiguity is too high, or no tags are seen
  private boolean shouldRejectPose(VisionIO.PoseObservation obs) {
    return obs.tagCount() == 0 
        || (obs.tagCount() == 1 && obs.ambiguity() > 0.3)
        || obs.avgTagDist() > 8.0;
  }

  /*
   * don't want to couple drive and vision at the moment
   * consumer interface that intakes information and sends to drive subsystem
   */
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}