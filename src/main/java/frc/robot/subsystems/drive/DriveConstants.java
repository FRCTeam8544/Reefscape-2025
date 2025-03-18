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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // Driving Parameters
  public static final double maxSpeedMetersPerSec = 4.8 / 3.0; // TODO for safety
  public static final double odometryFrequency = 100.0; // Hz

  // Chassis configuration
  public static final double trackWidth = Units.inchesToMeters(23.5); // 27 inches wide
  // Distance between centers of right and left wheels on robot
  public static final double wheelBase = Units.inchesToMeters(29.0); // 32.5 inches long
  // Distance between front and back wheels on robot

  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // frontLeft
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // frontRight
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // backLeft
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // backRight
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

  // Spark Device CAN IDs
  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  // Drive motor configuration
  // See https://www.revrobotics.com/rev-21-1652/
  public static final double freeSpeedRpm = 6784;
  public static final int driveMotorCurrentLimit = 50;

  // The MAXSwerve module can be configured with one of three pinion gears: 12T,
  // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
  // more teeth will result in a robot that drives faster).
  public static final int drivingMotorPinionTeeth = 14;

  // Calculations required for driving motor conversion factors
  public static final double drivingMotorFreeSpeedRps = freeSpeedRpm / 60;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double wheelDiameterMeters = 2 * wheelRadiusMeters;
  public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
  public static final double driveMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
  public static final double driveWheelFreeSpeedRps =
      (drivingMotorFreeSpeedRps * wheelCircumferenceMeters) / driveMotorReduction;
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double drivingFactor = wheelDiameterMeters * Math.PI / driveMotorReduction;
  public static final double driveEncoderPositionFactor = drivingFactor;
  public static final double driveEncoderVelocityFactor = drivingFactor / 60.0;

  // Drive PID configuration
  // Drive motor is really sensitive, keep pids small.
  public static final double driveKp = 0.00008; // 2e-4 okish
  public static final double driveKd = 0.000004;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.05;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
