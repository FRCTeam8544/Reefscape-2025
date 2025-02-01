// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;
import edu.wpi.first.wpilibj.XboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawWrist extends Command {
  ClawIntake clawIntake;
  XboxController juliet;

  public ClawWrist(ClawIntake clawIntake, XboxController juliet) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() { // Called when the command is initially scheduled.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(juliet.getXButtonPressed()){
      clawIntake.wristTurn(true);}
      else{clawIntake.wrist.set(0);}

    if(juliet.getYButtonPressed()){
      clawIntake.wristTurn(false);}
      else{clawIntake.wrist.set(0);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      clawIntake.wrist.set(0);}
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
