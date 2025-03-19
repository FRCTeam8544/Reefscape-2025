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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  public static final int missedUpdateLimit = 500; // 250 updates per second
  
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); // FRC NE
     //AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // Worlds
 //     AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // TODO Define custom school field tag setup... and a way to switch??

  public static final int INVALID_APRIL_TAG = -1;

  // Camera names, must match names configured on coprocessor
  public static String leftChassisApriltag = "leftChassisApriltag";
  public static String rightChassisApriltag = "rightChassisApriltag";
  public static String driverCam = "driverCam";

  // Robot to camera transforms - Need to be configured relative to gyro

  // Plywood prototype values
  // 18 and 15 1/2 from the robot front left corner
  // 7/8 either way from stalk to lens center
  // the transform should be in meters.
  // These values are centered on the "robot center"
  // Gyro will be offset from this.
  // Left Chasis
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(8.0),
          Units.inchesToMeters(1.1250),
          Units.inchesToMeters(23.5),
          new Rotation3d(0.0, 0, 0.0));
  // Right Chasis
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(8.0),
          Units.inchesToMeters(-2.8750),
          Units.inchesToMeters(23.5),
          new Rotation3d(0.0, 0, 0.0));
  // Driver
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(12.0),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(10.5),
          new Rotation3d(0.0, 0, 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // leftChassisApriltag
        1.0, // rightChassisApriltag
        1.0 // driverCam
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
