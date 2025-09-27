package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

public class transversepose extends Command{
    Elevator elevator;
    ClawIntake clawIntake;
    double elbpos = 0.25;
    double wristpos = 0.0;

    public transversepose(Elevator elevator, ClawIntake clawIntake){
        this.clawIntake = clawIntake;
        this.elevator = elevator;
        addRequirements(elevator);
        addRequirements(clawIntake);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        elevator.turnElbowToPosition(elbpos);
        clawIntake.turnWristToPosition(wristpos);
    }

    @Override
    public void end(boolean interrupted){
        elevator.turnElbowToPosition(elevator.getElbowPos());
        clawIntake.turnWristToPosition(clawIntake.getPos());
    }

    @Override
    public boolean isFinished(){
        //return (elevator.getElbowPos() < elbpos + 0.05 && elevator.getElbowPos() > elbpos - 0.05 && clawIntake.getPos() > wristpos - 0.05 && clawIntake.getPos() < wristpos + 0.05);
        return (elevator.getElbowPos() < elbpos + 0.02 && elevator.getElbowPos() > elbpos - 0.02);
    }
    
}
