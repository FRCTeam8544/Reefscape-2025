// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawIntake extends SubsystemBase {
  /** Creates a new ClawIntake. */
  private static SparkMax roller = new SparkMax(12, null);

  private static SparkMax wrist = new SparkMax(13, null);
  private SparkMaxConfig config = new SparkMaxConfig();

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
