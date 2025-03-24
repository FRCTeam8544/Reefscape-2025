package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;

public class RollerAuto extends Command{//this file is dedicated to SaintLaurentYSL by Lil Yachty and Lil Baby


    ClawIntake clawIntake;

    public RollerAuto(ClawIntake clawIntake){//I love ysl
        this.clawIntake = clawIntake;
        addRequirements(clawIntake);
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        clawIntake.rollerRoll(true);
    }

    @Override
    public void end(boolean interrupted){
        clawIntake.rollerRoll(false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
