// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElbowForward extends Command {
  Elevator elevator;
  CommandXboxController juliet;
  Trigger rightBackTop;

  public ElbowForward(Elevator elevator, CommandXboxController juliet, Trigger rightBackTop) {
    this.elevator = elevator;
    this.juliet = juliet;
    this.rightBackTop = rightBackTop;

    addRequirements(elevator);

    rightBackTop = juliet.rightTrigger();
  }

  @Override
  public void initialize() { // Called when the command is initially scheduled.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rightBackTop.getAsBoolean()) {
      elevator.spinElbowForward(true);} 
    else {
      elevator.spinElbowForward(false);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
