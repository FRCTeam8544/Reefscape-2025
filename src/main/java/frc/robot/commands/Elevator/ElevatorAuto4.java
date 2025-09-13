package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutosReal.GildedPoseClawAndElbow;
import frc.robot.commands.AutosReal.transversepose;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;

public class ElevatorAuto4 extends SequentialCommandGroup{

    public ElevatorAuto4(Elevator elevator, ClawIntake clawIntake){
        addCommands(
            new SequentialCommandGroup(
                new transversepose(elevator).withTimeout(0.5),
                new ElevatorPose(elevator, 0.2, 9.31),
                new ElbowPose(elevator, 0.1, 0.2984) //0.366
            )
        );
    }
    
}
