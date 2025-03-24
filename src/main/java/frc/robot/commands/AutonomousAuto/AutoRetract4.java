// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ClawIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRetract4 extends Command {
  Elevator elevator;
  ClawIntake clawIntake;

  public AutoRetract4(Elevator elevator, ClawIntake clawIntake) {
    this.elevator = elevator;
    this.clawIntake = clawIntake;

    addRequirements(elevator, clawIntake);
  }

  public void wristAutoRetract() {
    //clawIntake.turnWristToPosition(0.1); // TODO add start position, only call once
    //if(ClawIntake.wristEncoder.getPosition() > 0.1) {clawIntake.wrist(-.3);} 
   // else{clawIntake.wrist.set(0);}
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristAutoRetract();
    elevator.elevatorLow(true);
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
