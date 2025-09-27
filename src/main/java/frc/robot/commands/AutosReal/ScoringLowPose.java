package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

public class ScoringLowPose extends Command{

    Elevator elevator;
    ClawIntake clawIntake;

    public ScoringLowPose(Elevator elevator, ClawIntake clawIntake){
        this.elevator = elevator;
        this.clawIntake = clawIntake;
        addRequirements(elevator, clawIntake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
      /*   if (elevator.getElbowPos() > 0.5 && !(elevator.getElbowPos() < 0.6)){
            elevator.spinElbowBackwards(true);
        }else if (elevator.getElbowPos() < 0.6 && !(elevator.getElbowPos() > 0.5)){
            elevator.spinElbowForward(true);
        }
        else{
            elevator.spinElbowForward(true);
        }
        if (clawIntake.getPos() > 0.7 && !(clawIntake.getPos() < 0.8)){
            clawIntake.wristTurnBack(true);
        }else if (clawIntake.getPos() < 0.8 && !(clawIntake.getPos() > 0.7)){
            clawIntake.wristTurn(true);
        }
        else{
            clawIntake.wristTurn(true);
        }*/
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return true;//(elevator.getElbowPos() < 0.6 && elevator.getElbowPos() > 0.5 && clawIntake.getPos() > 0.8 && clawIntake.getPos() < 0.9);
    }
    
}
