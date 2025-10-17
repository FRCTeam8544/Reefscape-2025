// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.LogUtil;

import java.util.Optional;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GameConstants;
import frc.robot.subsystems.vision.VisionConstants;


public class Navigation extends SubsystemBase {
  
  private Alliance alliance;
  private int currentTargetTag = -1; // No target
  private double bestApproachAngle = 0.0;

  private Supplier<Pose2d> robotPoseSupplier;

  // Reef data
  private ArrayList<Pose2d> reefFacePoses = new ArrayList<Pose2d>();
  private ArrayList<Rotation2d> reefWedgeAngles = new ArrayList<Rotation2d>();
  private ArrayList<Integer> reefWedgeTags = new ArrayList<Integer>();
  private Translation2d reefCentroid;

  public DoubleSupplier approachAngleSupplier =
      () -> {
        return bestApproachAngle;
      };


  public Navigation(Supplier<Pose2d> poseSupplier, Alliance alliance) {
    this.robotPoseSupplier = poseSupplier;
    this.alliance = alliance;
    
    initializeGameRegions(alliance);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //navigationIO.updateInputs(climberInOutData);

    final Pose2d currentPose = robotPoseSupplier.get();
 
    // Determine which tag should be targeted
    currentTargetTag = getBestReefTarget(currentPose);

    // Determine best angle to approach the target given the current position
    Optional<Pose2d> targetTagOption = getTagPose(currentTargetTag);
    if (targetTagOption.isPresent()) {
        bestApproachAngle = getBestApproachAngle(targetTagOption.get()).getRadians();
    }

    // Log summary data
    // log current pose, selected tag, approach angle
    //LogUtil.logData("Climber", climberInOutData);
  }


  // Determine which reef face wedge contains the robot and provide the tag associated with that face.
  private int getBestReefTarget(Pose2d robotPose)
  {
    int selectedTag = -1;

    // Robot minus reef to get the "vector" from reef centroid to the robot
    // The vector angle relative to the field origin can be computed.
    // This angle will be used to select from the reef angle wedges, one per face,
    // like a clock where noon is down field and three o'clock is on your left...
    // not sure this helps...anyways
    Translation2d clockVector = robotPose.getTranslation().minus(reefCentroid);
    Rotation2d clockAngle = clockVector.getAngle();

    // WpiLib rotations are counter clockwise (zero lies on the positive x axis)
    if ( (clockAngle.getRadians() <= reefWedgeAngles.get(0).getRadians()) &&
         (clockAngle.getRadians() > reefWedgeAngles.get(5).getRadians()) ) {
        currentTargetTag = reefWedgeTags.get(0);
    }
    else if ( (clockAngle.getRadians() <= reefWedgeAngles.get(1).getRadians()) &&
              (clockAngle.getRadians() > reefWedgeAngles.get(0).getRadians()) ) {
        currentTargetTag = reefWedgeTags.get(1);
    }
    else if ( (clockAngle.getRadians() <= reefWedgeAngles.get(2).getRadians()) &&
              (clockAngle.getRadians() > reefWedgeAngles.get(1).getRadians()) ) {
        currentTargetTag = reefWedgeTags.get(2);
    }
    else if ( (clockAngle.getRadians() <= reefWedgeAngles.get(3).getRadians()) &&
              (clockAngle.getRadians() > reefWedgeAngles.get(2).getRadians()) ) {
        currentTargetTag = reefWedgeTags.get(2);
    }
    else if ( (clockAngle.getRadians() <= reefWedgeAngles.get(4).getRadians()) &&
              (clockAngle.getRadians() > reefWedgeAngles.get(3).getRadians()) ) {
        currentTargetTag = reefWedgeTags.get(3);
    }
    else if ( (clockAngle.getRadians() <= reefWedgeAngles.get(5).getRadians()) &&
              (clockAngle.getRadians() > reefWedgeAngles.get(4).getRadians()) ) {
        currentTargetTag = reefWedgeTags.get(4);
    }
    return selectedTag;
  }

  // Get the Pose2d for the given tag number if it exists
  private Optional<Pose2d> getTagPose(int tagNumber)
  {
    Optional<Pose3d> tagOption = VisionConstants.aprilTagLayout.getTagPose(tagNumber);
    if (tagOption.isPresent()) {
        Pose2d  thePose = new Pose2d(tagOption.get().getX(),
                                     tagOption.get().getY(), 
                                     tagOption.get().getRotation().toRotation2d());
        return Optional.of(thePose);
    }
    else {
        return Optional.empty();
    }
  }

  // Return a field relative rotation that the robot should use to "aim" at a target
  private Rotation2d getBestApproachAngle(Pose2d targetPose)
  {
    // Add filtering on position to avoid tag flipping... TODO

    // Assume that tag pose "facing out" from field element, so flip it to be as the robot sees it.
    return targetPose.getRotation().unaryMinus();
  }

  // TODO This is game specific and may belong elsewhere
  private void initializeGameRegions(Alliance alliance)
  {
  // TODO add other areas of interest...
    populateReefData();
  }

  // Determine the centroid (field reference frame, blue origin) and tag poses of the provided alliance's reef
  private void populateReefData()
  {
    Double xCentroid = 0.0;
    double yCentroid = 0.0;
    double tagCount = 0.0;
    if (alliance == Alliance.Blue) {
        for (int tagIdx = 0; tagIdx < GameConstants.numReefFaceTags; ++tagIdx)
        {
            final int lookupTag = GameConstants.blueReefAprilTagIDs[tagIdx];
            Optional<Pose3d> tagOption = VisionConstants.aprilTagLayout.getTagPose(lookupTag);
            if (tagOption.isPresent()) {
                xCentroid += tagOption.get().getX();
                yCentroid += tagOption.get().getY();
                tagCount += 1.0;

                reefFacePoses.add(getTagPose(lookupTag).get());
            }
           // tagOption.ifPresent(thePose -> { xCentroid += thePose.getX();
            //                                 yCentroid += thePose.getY();
             //                                tagCount += 1.0; });
        }
    }
    else
    {
        for (int tagIdx = 0; tagIdx < GameConstants.numReefFaceTags; ++tagIdx)
        {
            final int lookupTag = GameConstants.redReefAprilTagsIDs[tagIdx];
            Optional<Pose3d> tagOption = VisionConstants.aprilTagLayout.getTagPose(lookupTag);
            if (tagOption.isPresent()) {
                xCentroid += tagOption.get().getX();
                yCentroid += tagOption.get().getY();
                tagCount += 1.0;

                reefFacePoses.add(getTagPose(lookupTag).get());
            }
        }
    }

    if (tagCount > 0.0) {
        xCentroid = xCentroid / tagCount;
        yCentroid = yCentroid / tagCount;
    }
    reefCentroid = new Translation2d(xCentroid, yCentroid);

    // Build the wedge angles counter clockwise starting at positive X "down field from blue"
    Rotation2d currentFace = Rotation2d.kZero; // Starts with Down field face
    Rotation2d halfFaceAngle = Rotation2d.fromDegrees(360 / (2 * GameConstants.numReefFaceTags) );
    Rotation2d fullFaceAngle = Rotation2d.fromDegrees(60 / GameConstants.numReefFaceTags);
    for (int faceIdx = 0; faceIdx < GameConstants.numReefFaceTags; ++faceIdx) {
        reefWedgeAngles.add( currentFace.rotateBy(halfFaceAngle) );
        currentFace = currentFace.rotateBy( fullFaceAngle ); // Advance to next face counter clockwise from positive X
    }

    // Build reef wedge tag ids that match the face selection order from above
    if (alliance == Alliance.Blue) {
        reefWedgeTags.add(21);
        reefWedgeTags.add(20);
        reefWedgeTags.add(19);
        reefWedgeTags.add(18);
        reefWedgeTags.add(17);
        reefWedgeTags.add(22);
    }
    else {
        reefWedgeTags.add(7);
        reefWedgeTags.add(8);
        reefWedgeTags.add(9);
        reefWedgeTags.add(10);
        reefWedgeTags.add(11);
        reefWedgeTags.add(6);
    }
  }
}

