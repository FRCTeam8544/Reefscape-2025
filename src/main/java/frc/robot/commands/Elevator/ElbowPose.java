package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElbowPose extends Command{

    Elevator elevator;
    double tollerance;
    double position;
    public ElbowPose(Elevator elevator, double tollerance, double position){
        this.elevator = elevator;
        this.tollerance = tollerance;
        this.position = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        elevator.turnElbowToPosition(position);
    }

    @Override
    public void end(boolean end){
        elevator.turnElbowToPosition(elevator.getElbowPos());
    }
    
    @Override
    public boolean isFinished(){
        return ((elevator.getElbowPos() < tollerance + position) && (elevator.getElbowPos() > position - tollerance));
    }
}
