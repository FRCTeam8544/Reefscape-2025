package frc.robot.commands.AutosReal;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.ElevatorAuto2;
import frc.robot.commands.Elevator.ElevatorAuto3;
import frc.robot.commands.Elevator.ElevatorPose;
import frc.robot.commands.Elevator.IntakeAuto;
import frc.robot.commands.Elevator.sourceAuto;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;

public class LILYACHTY extends SequentialCommandGroup{

    public LILYACHTY(Elevator elevator, ClawIntake clawIntake, Drive drive){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1, 0, 0);
        addCommands(
            new SequentialCommandGroup(
                //new IntakeAuto(elevator, clawIntake).withTimeout(0.75),
                new ElevatorPose(elevator, 0.2, 1.4),
                new freakyahdriveauto(drive, chassisSpeeds).withTimeout(0.85),
                new freakyahElbowAuto(elevator).withTimeout(0.75),
                //new RollerAuto(clawIntake)
                new asdfghjk(clawIntake).withTimeout(1)
            )
        );
    }
    
}
