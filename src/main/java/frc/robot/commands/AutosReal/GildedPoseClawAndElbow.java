package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

public class GildedPoseClawAndElbow extends Command{

    Elevator elevator;
    ClawIntake clawIntake;
    double tollerance;
    double elbPos;
    double clawPos;

    public GildedPoseClawAndElbow(Elevator elevator, ClawIntake clawIntake, double tollerance, double elbPos, double clawPos){
        this.elevator = elevator;
        this.clawIntake = clawIntake;
        this.tollerance = tollerance;
        this.elbPos = elbPos;
        this.clawPos = clawPos;
       addRequirements(elevator, clawIntake);
       
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        elevator.turnElbowToPosition(elbPos);
        clawIntake.turnWristToPosition(clawPos);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return (elevator.getElbowPos() < elbPos + tollerance && elevator.getElbowPos() > elbPos - tollerance && clawIntake.getPos() > clawPos - tollerance && clawIntake.getPos() < clawPos + tollerance);
    }
    
}
