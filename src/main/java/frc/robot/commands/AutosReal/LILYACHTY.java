package frc.robot.commands.AutosReal;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.ElevatorAuto2;
import frc.robot.commands.Elevator.ElevatorAuto3;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;

public class LILYACHTY extends SequentialCommandGroup{

    public LILYACHTY(Elevator elevator, ClawIntake clawIntake, Drive drive){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1, 0, 0);
        addCommands(
            new SequentialCommandGroup(
                new ElevatorAuto3(elevator, clawIntake),
                new freakyahdriveauto(drive, chassisSpeeds).withTimeout(1),
                new RollerAuto(clawIntake)
            )
        );
    }
    
}
