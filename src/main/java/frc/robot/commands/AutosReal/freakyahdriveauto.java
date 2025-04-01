package frc.robot.commands.AutosReal;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class freakyahdriveauto extends Command{
    Drive drive;
    ChassisSpeeds chassisSpeeds;

    public freakyahdriveauto(Drive drive, ChassisSpeeds chassisSpeeds){
        this.drive = drive;
        this.chassisSpeeds = chassisSpeeds;
        addRequirements(drive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        drive.runVelocity(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
