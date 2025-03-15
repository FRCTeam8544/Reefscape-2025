// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {

  private static final double ELEVATOR_DEADBAND = 0.15;
  private static final double ELBOW_DEADBAND = 0.3;

  /** Command elevator using joysticks (controlling linear and angular velocities). */
  public static Command joystickElevator(
      Elevator elevator, DoubleSupplier verticalSupplier, DoubleSupplier tiltSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(
                  verticalSupplier.getAsDouble(), tiltSupplier.getAsDouble());
                 
          // Convert to elevator relative speeds
          final double elevatorVelocity =
              linearVelocity.getX();
          final double elbowVelocity = MathUtil.applyDeadband(
              linearVelocity.getY(),
              ELBOW_DEADBAND);

          elevator.runElevatorVelocity(elevatorVelocity);
          if (elbowVelocity >= 0) {
            elevator.spinElbowForward(elbowVelocity != 0.0);
          }
          else {
            elevator.spinElbowBackwards(true);
          }
         // elevator.runWristVelocity(elbowVelocity);
        },
        elevator);
  }

  // x and y are joystick relative coordinates??? x (left/right) y (up/down)
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), ELEVATOR_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
