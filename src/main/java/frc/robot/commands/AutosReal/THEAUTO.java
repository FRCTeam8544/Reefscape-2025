package frc.robot.commands.AutosReal;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousAuto.FAKEIntakeAuto;
import frc.robot.commands.Elevator.ElevatorAuto4;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;//you cant just stay perchance

public class THEAUTO extends SequentialCommandGroup{//it insists upon itself doesnt it
    
    public THEAUTO(Elevator elevator, ClawIntake clawIntake, Drive drive){//Java is a language of subtlety 
        addCommands(
            new SequentialCommandGroup(
                //new ParallelCommandGroup(
                    new DriveAuto(drive).withTimeout(0.5)
                    //new ElevatorAuto4(elevator, clawIntake)
                //),
                // new RollerAuto(clawIntake).withTimeout(0.5),
                // new ParallelCommandGroup(
                //     new FAKEIntakeAuto(elevator, clawIntake),//brings everything down and ready for game :)
                //     new lilDriveBackAuto(drive).withTimeout(0.5)
                //)
            )
        );
    }
    
}
