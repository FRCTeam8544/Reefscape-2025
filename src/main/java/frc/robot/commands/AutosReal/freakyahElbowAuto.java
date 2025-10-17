package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class freakyahElbowAuto extends Command{
    Elevator elevator;

    public freakyahElbowAuto(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
   //     elevator.spinElbowBackwards(true);
    }

    @Override
    public void end(boolean interupted){
    //    elevator.spinElbowForward(false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
