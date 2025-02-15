// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private static SparkMax pusherRight =
      new SparkMax(Constants.climberConstants.climberCANID, MotorType.kBrushless);

  private static SparkMax pusherLeft =
      new SparkMax(Constants.climberConstants.climber2CANID, MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();

  public Climber() {
    pusherRight.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);

    pusherLeft.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);
    config.follow(Constants.climberConstants.climberCANID, true);
  }

  public void climberClimb(boolean go) {
    if (go) {
      pusherRight.set(.1);
    } else {
      pusherRight.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
