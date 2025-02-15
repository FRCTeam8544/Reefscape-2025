// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorUp extends Command {
  CommandXboxController juliet;
  elevator elevator;
  Trigger yButton = juliet.y();

  public elevatorUp(elevator elevator, CommandXboxController juliet, Trigger yButton) {
    this.elevator = elevator;
    this.juliet = juliet;
    this.yButton = yButton;
  }

  @Override
  public void initialize() {
    // called when command is initially scheduled
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (yButton.getAsBoolean() && !elevator.upStop.getAsBoolean()) {
      elevator.elevatorMove(true);
    } else {
      elevator.elevatorMove(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      elevator.elevatorMove(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
