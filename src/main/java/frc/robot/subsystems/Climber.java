// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.AbsoluteEncoder;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static SparkMax pusherRight = new SparkMax(Constants.climberConstants.climberCANID, MotorType.kBrushless);
  public static SparkMax pusherLeft = new SparkMax(Constants.climberConstants.climber2CANID, MotorType.kBrushless);
  private static SparkMaxConfig rightConfig = new SparkMaxConfig();
  private static SparkMaxConfig leftConfig = new SparkMaxConfig();
  private AbsoluteEncoder encoder;

  public Climber() {
    rightConfig.idleMode(IdleMode.kBrake);
    pusherRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightConfig.inverted(true);

    leftConfig.follow(Constants.climberConstants.climberCANID);
    leftConfig.idleMode(IdleMode.kBrake);
    pusherLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climberClimb(boolean go) {
    if (go && encoder.getPosition() > 4) {pusherRight.set(.1);} 
    else {pusherRight.set(0);}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
