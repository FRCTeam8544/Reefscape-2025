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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;


public class ElevatorCommands {

  private static final double ELEVATOR_DEADBAND = 0.1;
  private static final double ELBOW_DEADBAND = 0.2;

  private static int snapCount = 0;
  private static int clawSnapCount = 0;

  public static Command logPose(Elevator elevator, String prefix) {
    return Commands.run(
        () -> { elevator.logPose(prefix, snapCount++); }, elevator);
  }

  public static Command logPose(ClawIntake claw, String prefix) {
    return Commands.run(
        () -> { claw.logPose(prefix, clawSnapCount++); }, claw);
  }

  public static Command logPose(Climber climber, String prefix) {
    return Commands.run(
        () -> { climber.stopClimber(); climber.logPose(prefix, 0); }, climber);
  }
  
  /** Command elevator using joysticks (controlling linear and angular velocities). */
  public static Command joystickElevator(
      Elevator elevator, DoubleSupplier verticalSupplier, 
               Trigger elbowForwardTrigger, Trigger elbowBackwardTrigger) {
    return Commands.run(
        () -> {
          // Get linear velocity
          // Translation2d linearVelocity =
          //     getLinearVelocityFromJoysticks(
          //         verticalSupplier.getAsDouble(), tiltSupplier.getAsDouble());
                 
          // Convert to elevator relative speeds
          final double elevatorStickVelocity = MathUtil.applyDeadband(
              verticalSupplier.getAsDouble(),
              ELEVATOR_DEADBAND);
          // final double elbowStickVelocity = MathUtil.applyDeadband(
          //     linearVelocity.getY(),
          //     ELBOW_DEADBAND);

          final double dE = elevatorStickVelocity > 0 ? elevatorStickVelocity * 0.3 : elevatorStickVelocity * 0.1;

          // Apply velocities

          double pos = elevator.getElevatorPosition();
          double dE_apply_StartRegion = 0.75;
          double dE_apply_StopRegion = 9.3; //TODO update upper limit for joystick
          if ((pos >= dE_apply_StartRegion && dE < 0) || (pos <= dE_apply_StopRegion && dE > 0)) {
            elevator.runElevatorToPosition(pos + dE);
          }
          else {
            // TODO allow joystick to end limits
          }
          // }else if (pos >= 2.75 && dE > 0){
          //   elevator.runElevatorToPosition(3);
          // }else if(pos <= 0.75 && dE < 0){
          //   elevator.runElevatorToPosition(0.5);
          // }
          
          final boolean elbowForward = elbowForwardTrigger.getAsBoolean();
          final boolean elbowBackward = elbowBackwardTrigger.getAsBoolean();
          // if ( (elbowForward && elbowBackward) ) {
          //   elevator.spinElbowForward(false); // Stop if inputs conflict
          // }
          // else {
          //    elevator.spinElbowForward(elbowForward);
          //    elevator.spinElbowBackwards(elbowBackward);
          // }
          // if (elbowBackward){elevator.spinElbowBackwards(true);}
          // else if (elbowForward){elevator.spinElbowForward(true);}
          // else{elevator.spinElbowForward(false);}
          if (elbowBackward ^ elbowForward){
            double skibidi = elevator.getElbowPos();
            // double quanMillz = elbowForward? 0.05 : -0.05;
            double quanMillz = elbowForward? 0.1 : -0.1;

            elevator.sigmasigmaonthewall(skibidi + quanMillz);
          }else{
            elevator.sigmasigmaonthewall(elevator.getElbowPos());
          }
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
