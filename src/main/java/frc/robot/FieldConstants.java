// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }
  // contains the measurements and starting points for the cages; contains measurements from INCHES
  // to METERS
  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));
    public static final Translation2d farCageLow =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(30.221));
    public static final Translation2d middleCageLow =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(74.145));
    public static final Translation2d closeCageLow =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(117.053));
    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public static final Pose2d[] ReefScoringPositions = getReefScoringPositions();
  public static final Pose2d[] ReefCenterPositions = getCenterScoringPositions();

  // The distance backward from the reef that the center of the robot should be when scoring
  public static final double distanceBackFromReef = 0.67;
  public static final double reefFaceCenterToScoreDistance = 0.164338;

  // Calculates all the reef scoring positions using the centers of the faces as a reference
  public static Pose2d[] getReefScoringPositions() {
    Pose2d[] positions = new Pose2d[12];

    for (int i = 0; i < 12; i++) {
      // Use center of face as a reference and rotate it 180 degrees to point towards reef instead
      // of away
      Pose2d centerFacePose = Reef.centerFaces[i / 2];
      Pose2d scoringPose =
          new Pose2d(
              centerFacePose.getTranslation(),
              centerFacePose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

      // Alternate between translating each position to the left and right of the center of the face
      switch (i % 2) {
        case 0:
          scoringPose =
              translateCoordinates(
                  scoringPose,
                  centerFacePose.getRotation().getDegrees() + 90,
                  reefFaceCenterToScoreDistance);
          break;
        case 1:
          scoringPose =
              translateCoordinates(
                  scoringPose,
                  centerFacePose.getRotation().getDegrees() - 90,
                  reefFaceCenterToScoreDistance);
          break;
      }

      // Move position backwards
      positions[i] =
          translateCoordinates(
              scoringPose, centerFacePose.getRotation().getDegrees(), distanceBackFromReef);
    }

    return positions;
  }

  public static Pose2d[] getCenterScoringPositions() {
    Pose2d[] positions = new Pose2d[6];

    for (int i = 0; i < 6; i++) {
      Pose2d scoringPose =
          new Pose2d(
              Reef.centerFaces[i].getTranslation(),
              Reef.centerFaces[i].getRotation().rotateBy(Rotation2d.fromDegrees(180)));

      positions[i] =
          translateCoordinates(
              scoringPose, Reef.centerFaces[i].getRotation().getDegrees(), distanceBackFromReef);
    }
    return positions;
  }

  // Translates the coordinates of originalPose by some distance at some angle
  // Stolen- uhhh taken inspiration from team 6964 BearBots
  public static Pose2d translateCoordinates(
      Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  public static final Pose2d[] STATION_POSITION = getStationPositions();

  public static final double distanceBackFromStation = 0.72;

  // Returns an array of positions using the station center faces and moving the poses back
  public static Pose2d[] getStationPositions() {
    Pose2d[] stationPositions = new Pose2d[2];

    // Set poses to station center faces
    stationPositions[0] = CoralStation.leftCenterFace;
    stationPositions[1] = CoralStation.rightCenterFace;

    // Move each pose back and flip it 180 degrees
    for (int i = 0; i < 2; i++) {
      stationPositions[i] =
          translateCoordinates(
              stationPositions[i],
              stationPositions[i].getRotation().getDegrees(),
              distanceBackFromStation);
      stationPositions[i] =
          new Pose2d(
              stationPositions[i].getTranslation(),
              stationPositions[i].getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

    return stationPositions;
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }

  public static class StartingPositions {
    public static final Pose2d startPos1 =
        new Pose2d(
            startingLineX - Units.inchesToMeters(15),
            Barge.farCage.getY(),
            Rotation2d.fromDegrees(180));
    public static final Pose2d startPos2 =
        new Pose2d(
            startingLineX - Units.inchesToMeters(15),
            Barge.middleCage.getY(),
            Rotation2d.fromDegrees(180));
    public static final Pose2d startPos3 =
        new Pose2d(
            startingLineX - Units.inchesToMeters(15),
            Barge.closeCage.getY(),
            Rotation2d.fromDegrees(180));
    public static final Pose2d startPos4 =
        new Pose2d(
            startingLineX - Units.inchesToMeters(15),
            Barge.closeCageLow.getY(),
            Rotation2d.fromDegrees(180));
    public static final Pose2d startPos5 =
        new Pose2d(
            startingLineX - Units.inchesToMeters(15),
            Barge.middleCageLow.getY(),
            Rotation2d.fromDegrees(180));
    public static final Pose2d startPos6 =
        new Pose2d(
            startingLineX - Units.inchesToMeters(15),
            Barge.farCageLow.getY(),
            Rotation2d.fromDegrees(180));
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  // public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;//
  public static final int aprilTagCount = 22;

  /*
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official");

    AprilTagLayoutType(String name) {
      if (layout == null) {
        layoutString = "";
      } else {
        try {
          layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
          throw new RuntimeException(
              "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
        }
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
    */
}
