// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.AutosReal.ElevatorPose3;
import frc.robot.commands.AutosReal.GildedPoseClawAndElbow;
import frc.robot.commands.AutosReal.ScoringLowPose;
import frc.robot.commands.AutosReal.transversepose;
import frc.robot.subsystems.ClawIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAuto3 extends SequentialCommandGroup {
  /** Creates a new ElevatorAuto3. */

 
  public ElevatorAuto3(Elevator elevator, ClawIntake clawIntake) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
         
        new transversepose(clawIntake, elevator).withTimeout(0.5),
        new ElevatorPose(elevator, 0.1, 6.6),
        new GildedPoseClawAndElbow(elevator, clawIntake, 0.1, 0.12, 0.97)
      )
    );
  }
}
