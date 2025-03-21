// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private PhotonPipelineResult result = null;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  public double getBestTargetAbiguitity() {
    return result.getBestTarget().getPoseAmbiguity();
  }

  public int getBestTargetID() {
    int id = result.getBestTarget().getFiducialId();
    SmartDashboard.putNumber("Current Target ID", id);
    String targetLocationName = null;
    switch(id) {
      case 19:
          targetLocationName = "Blue Reef 1";
        case 18: 
          targetLocationName = "Blue Reef 2";
        case 17:
            targetLocationName = "Blue Reef 3";
        case 22:
            targetLocationName = "Blue Reef 4";
        case 21:
             targetLocationName = "Blue Reef 5";
        case 20:
             targetLocationName = "Blue Reef 6";
        case 6:
              targetLocationName = "Red Reef 1";
        case 7:
              targetLocationName = "Red Reef 2";   
        case 8:
              targetLocationName = "Red Reef 3";
        case 9:
             targetLocationName = "Red Reef 4"; 
        case 10:
             targetLocationName = "Red Reef 5";
        case 11:
             targetLocationName = "Red Reef 6";
        case 13:
              targetLocationName = "Blue Source 1";
         case 12: 
              targetLocationName = "Blue Source 2";
         case 1:
              targetLocationName = "Red Source 1";
         case 2:
               targetLocationName = "Red Source 2";
    }
    SmartDashboard.putString("Current Target Tag", targetLocationName);
    return id;
  }
}
