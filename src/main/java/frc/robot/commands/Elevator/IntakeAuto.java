package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutosReal.GildedPoseClawAndElbow;
import frc.robot.commands.AutosReal.transversepose;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

public class IntakeAuto extends SequentialCommandGroup{

    public IntakeAuto(Elevator elevator, ClawIntake clawIntake){
        addCommands(
            new SequentialCommandGroup(
                new transversepose(elevator, clawIntake).withTimeout(0.5),
                new GildedPoseClawAndElbow(elevator, clawIntake, 0.02, 0.4, -0.17),
                new ElevatorPose(elevator, 0.2, 1.78)//2.1
                //new ElbowPose(elevator, 0, 0.45) // TODO finalize values above from data...
            )
        ); 
    }
    
}
