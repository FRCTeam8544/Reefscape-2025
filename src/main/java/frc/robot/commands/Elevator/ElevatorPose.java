package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPose extends Command{
    Elevator elevator;
    double pos;
    double tollerance;

  public ElevatorPose(Elevator elevator, double tollerance, double pos) {
    this.elevator = elevator;
    this.pos = pos;
    this.tollerance = tollerance;

    addRequirements(elevator);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runElevatorToPosition(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getElePos() < pos + tollerance && elevator.getElePos() > pos - tollerance;
  }
    
}
