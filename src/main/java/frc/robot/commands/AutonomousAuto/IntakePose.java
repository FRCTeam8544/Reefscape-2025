// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePose extends Command {
  ClawIntake clawIntake;
  Elevator elevator;
  public IntakePose(ClawIntake clawIntake, Elevator elevator) {
    this.clawIntake = clawIntake;
    this.elevator = elevator;

    addRequirements(clawIntake, elevator);
  }

  public void intakePose(){
    clawIntake.turnWristToPosition(0);
    //if(clawIntake.wristEncoder.getPosition() > 0) {clawIntake.wrist.set(-.3);}
   // else {clawIntake.wrist.set(0);}
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    intakePose();
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
