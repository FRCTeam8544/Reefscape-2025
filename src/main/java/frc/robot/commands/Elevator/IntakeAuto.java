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
                //new transversepose(clawIntake, elevator).withTimeout(0.5),
                new ElevatorPose(elevator, 0.2, 1.6)
                //new GildedPoseClawAndElbow(elevator, clawIntake, 0.05, 0.05, 0.76)
            )
        );
    }
    
}
