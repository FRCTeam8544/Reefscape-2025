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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands extends Command {

  private static final double DEADBAND = 0.1;

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
          double elevatorVelocity =
              linearVelocity.getX(); // * elevator.getMaxLinearSpeedMetersPerSec();
          double wristVelocity =
              linearVelocity.getY(); // * elevator.getMaxLinearSpeedMetersPerSec();

          // TODO tie elevator and wrist into one joystick?
        //  elevator.runElevatorVelocity(elevatorVelocity);
         // elevator.runWristVelocity(wristVelocity);
        },
        elevator);
  }

  // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //  if (!elevator.downStop.getAsBoolean() ) {
  ///    elevator.elevatorLow(true);
  // } else {
  //   elevator.elevatorLow(false);
  //  }
  // }

  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //  elevator.elevatorLow(false);
  // }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }

  // x and y are joystick relative coordinates??? x (left/right) y (up/down)
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
