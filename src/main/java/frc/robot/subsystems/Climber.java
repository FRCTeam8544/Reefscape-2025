// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private static SparkFlex pusher = new SparkFlex(16, MotorType.kBrushless);
  private static SparkFlexConfig config = new SparkFlexConfig();

  public Climber() {
    pusher.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);
  }
  public void climberClimb() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
