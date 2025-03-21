// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ClawIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator3Auto extends Command {
  /** Creates a new Elevator3Auto. */
  Elevator elevator;
  ClawIntake clawIntake;

  public Elevator3Auto() {
    this.elevator = elevator;
    this.clawIntake = clawIntake;

    addRequirements(elevator, clawIntake);
  }

  public void elevatorPosition(){elevator.runElevatorToPosition(7);} //idk whatever for all incase you see this
  public void elbowPosition(){elevator.turnElbowToPosition(0, .3);}
  public void wristPosition(){clawIntake.turnWristToPosition(0, 0);}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorPosition();
    elbowPosition();
    wristPosition();
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
