// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ClawIntake;
import frc.robot.commands.AutonomousAuto.AutoRollers;
import frc.robot.commands.AutonomousAuto.AutoRetract4;
import frc.robot.commands.AutonomousAuto.ScoreAuto4;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {

  public AutoScore(Elevator elevator, ClawIntake clawIntake) {

    addCommands(
      new SequentialCommandGroup(

        new ScoreAuto4(elevator, clawIntake).withTimeout(3), //timeout will end it

        new AutoRollers(clawIntake).withTimeout(1),

        new AutoRetract4(elevator, clawIntake).withTimeout(3)
      )
    );
  }
}
