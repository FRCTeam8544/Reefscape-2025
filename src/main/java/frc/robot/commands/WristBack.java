// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristBack extends Command {
  ClawIntake clawIntake;
  CommandXboxController juliet;
  Trigger leftBack, rightBack;

  public WristBack(ClawIntake clawIntake, CommandXboxController juliet, Trigger leftBack, Trigger rightBack) {
    this.clawIntake = clawIntake;
    this.juliet = juliet;
    this.leftBack = leftBack;
    this.rightBack = rightBack;

    leftBack = juliet.leftBumper();
    rightBack = juliet.rightBumper();
    addRequirements(clawIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   /* if (leftBack.getAsBoolean() && !clawIntake.wristBackwardStop.getAsBoolean()) {
      clawIntake.wristTurnBack(true);
    }else if (rightBack.getAsBoolean() && !clawIntake.wristForwardStop.getAsBoolean()){
      clawIntake.wristTurn(true);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
