// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;






public class LEDIdle extends Command {
  LEDs leds;


  public LEDIdle(LEDs leds ) {
    this.leds=leds;
    addRequirements(leds);


  }

  @Override
  public void initialize() { // Called when the command is initially scheduled.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      leds.violet();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public Command.InterruptionBehavior getInterruptionBehavior(){
    return Command.InterruptionBehavior.kCancelSelf;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
