package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawIntake;

public class asdfghjk extends Command{

    ClawIntake clawIntake;
    public asdfghjk(ClawIntake clawIntake){
        this.clawIntake = clawIntake;
        addRequirements(clawIntake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        clawIntake.rollerRollBack(true);
    }


    @Override
    public void end(boolean interrupted){
        clawIntake.rollerRollBack(false);

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
