// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRollerIntake extends Command {
  ClawIntake clawIntake;

  public AutoRollerIntake(ClawIntake clawIntake) {
    this.clawIntake = clawIntake;

    addRequirements(clawIntake);
  }

  public void rollersAutoIntake(boolean c) {
    if(c) {clawIntake.rollerRight.setVoltage(-7);}
    else {clawIntake.rollerRight.setVoltage(0);}
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollersAutoIntake(true);
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
