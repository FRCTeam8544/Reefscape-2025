// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawWrist extends Command {
  ClawIntake clawIntake;
  CommandXboxController juliet;
  Trigger rightBack;
  Trigger leftBack;

  public ClawWrist(
      ClawIntake clawIntake, CommandXboxController juliet, Trigger rightBack, Trigger leftBack) {
    this.clawIntake = clawIntake;
    this.juliet = juliet;
    this.rightBack = rightBack;
    this.leftBack = leftBack;

    leftBack = juliet.leftBumper();
    rightBack = juliet.rightBumper();
  }

  @Override
  public void initialize() { // Called when the command is initially scheduled.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rightBack.getAsBoolean() && !clawIntake.wristStop.getAsBoolean()) {
      clawIntake.wristTurn(true);
    } else {
      ClawIntake.wrist.set(0);
    }

    if (leftBack.getAsBoolean() && !clawIntake.wristStop.getAsBoolean()) {
      clawIntake.wristTurnBack(true);
    } else {
      ClawIntake.wrist.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      ClawIntake.wrist.set(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
