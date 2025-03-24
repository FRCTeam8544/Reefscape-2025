package frc.robot.commands.AutosReal;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class lilDriveBackAuto extends Command{//named after our robot lil yachty
    Drive drive;
    ChassisSpeeds chassisSpeeds;
    Pose2d pose;

    public lilDriveBackAuto(Drive drive){
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        chassisSpeeds.vyMetersPerSecond = -0.5;
        chassisSpeeds.vxMetersPerSecond = 0;
        pose = drive.getPose();
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
        return (drive.getPose().getMeasureY().abs(Meter) + 0.5 <= pose.getMeasureY().abs(Meter));
    }
    
}
