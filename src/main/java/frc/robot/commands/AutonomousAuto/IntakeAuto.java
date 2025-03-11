// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.AutonomousAuto.AutoRollerIntake;
import frc.robot.commands.AutonomousAuto.AutoRetract4;
import frc.robot.commands.AutonomousAuto.IntakePose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAuto extends SequentialCommandGroup {
  /** Creates a new IntakeAuto. */
  public IntakeAuto(Elevator elevator, ClawIntake clawIntake) {
    addCommands(
      new SequentialCommandGroup(
        new IntakePose(clawIntake, elevator).withTimeout(2), //time to be set but timeout will auto stop wherever
        new AutoRollerIntake(clawIntake).withTimeout(1),
        new AutoRetract4(elevator, clawIntake).withTimeout(3)
      )
    );
  }
}
