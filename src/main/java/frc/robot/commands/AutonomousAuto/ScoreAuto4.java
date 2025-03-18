// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ClawIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAuto4 extends Command {
  Elevator elevator;
  ClawIntake clawIntake;

  public ScoreAuto4(Elevator elevator, ClawIntake clawIntake) {
    this.elevator = elevator;
    this.clawIntake = clawIntake;

    addRequirements(elevator, clawIntake);
  }

   public void elevatorAuto4(){
    if (Elevator.encoder.getPosition() < 9.3 || !elevator.upStopHit) {elevator.runElevatorVelocity(.7);}
    else {elevator.runElevatorVelocity(0);}
    } //button map this too? like outake & elevator one button go score 

    public void elbowAutoScore() {
    if(Elevator.elbowEncoder.getPosition() > .3 || elevator.elbowEncoder.getPosition() < .3) 
        {Elevator.elbowController.set(.15);} //basically become number x (probs not 4)
       else{elevator.elbowController.set(0);}
      }

      public void wristAuto(){
        if(clawIntake.wristEncoder.getPosition() < .977) {clawIntake.wrist.set(.3);}
        else{clawIntake.wrist.set(0);}
      }    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorAuto4();
    elbowAutoScore();
    wristAuto();
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
