// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorDown extends Command {
  /** Creates a new elevatorDown. */
  CommandXboxController juliet;

  Elevator elevator;
  Trigger aButton;

  public elevatorDown(Elevator elevator, CommandXboxController juliet, Trigger aButton) {
    this.elevator = elevator;
    this.juliet = juliet;
    this.aButton = aButton;

    aButton = juliet.a();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!elevator.downStop.getAsBoolean() && aButton.getAsBoolean()) {
      elevator.elevatorLow(true);
    } else {
      elevator.elevatorLow(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorLow(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
