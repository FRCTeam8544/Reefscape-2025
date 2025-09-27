package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutosReal.GildedPoseClawAndElbow;
import frc.robot.commands.AutosReal.transversepose;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

public class sourceAuto extends SequentialCommandGroup{

    public sourceAuto(Elevator elevator, ClawIntake clawIntake){
        addCommands(
            new SequentialCommandGroup(
                new transversepose(elevator, clawIntake),
                new ElevatorPose(elevator, 0.1, 0.77),
                new GildedPoseClawAndElbow(elevator, clawIntake, 0.1, 0.7, 0.7)
            )
        );
    }
    
}
