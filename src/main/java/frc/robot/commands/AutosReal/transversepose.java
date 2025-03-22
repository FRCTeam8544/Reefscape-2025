package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

public class transversepose extends Command{

    ClawIntake clawIntake;
    Elevator elevator;
    double elbpos = 0.09;
    double wristpos = 0.96;

    public transversepose(ClawIntake clawIntake, Elevator elevator){
        this.clawIntake = clawIntake;
        this.elevator = elevator;
        addRequirements(clawIntake, elevator);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (elevator.getElbowPos() > elbpos - 0.1 && !(elevator.getElbowPos() < elbpos + 0.1)){
            elevator.spinElbowBackwards(true);
        }else if (elevator.getElbowPos() < elbpos + 0.1 && !(elevator.getElbowPos() > elbpos - 0.1)){
            elevator.spinElbowForward(true);
        }
        else{
            elevator.spinElbowForward(true);
        }
        if (clawIntake.getPos() < wristpos + 0.1 && !(clawIntake.getPos() > wristpos - 0.1)){
            clawIntake.wristTurnBack(true);
        }else if (clawIntake.getPos() < wristpos + 0.1 && !(clawIntake.getPos() > wristpos - 0.1)){
            clawIntake.wristTurn(true);
        }
        else{
            clawIntake.wristTurn(true);
        }
    }

    @Override
    public void end(boolean interrupted){
        elevator.spinElbowForward(false);
        clawIntake.wristTurn(false);
    }

    @Override
    public boolean isFinished(){
        return (elevator.getElbowPos() < elbpos + 0.1 && elevator.getElbowPos() > elbpos - 0.1 && clawIntake.getPos() > wristpos - 0.1 && clawIntake.getPos() < wristpos + 0.1);
    }
    
}
