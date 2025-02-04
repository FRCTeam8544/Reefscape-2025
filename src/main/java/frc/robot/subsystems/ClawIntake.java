// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawIntake extends SubsystemBase {
  /** Creates a new ClawIntake. */
  public static SparkFlex roller = new SparkFlex(12, MotorType.kBrushless);
  public static SparkFlex wrist = new SparkFlex(13, MotorType.kBrushless);
  private static SparkFlexConfig config = new SparkFlexConfig();

  public ClawIntake() {
    roller.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);

    wrist.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);
  }

  public void rollerRoll(boolean go) {
  if(go){roller.set(.5);}
  if(!go){roller.set(-.5);}
  }

  public void wristTurn(boolean forward){ 
    if(forward)wrist.set(.1);
    if(!forward)wrist.set(-.1); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
