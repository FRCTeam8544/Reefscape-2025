// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.MotorJointSparkMax;
import frc.robot.subsystems.MotorJointIOInputsAutoLogged;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static SparkMax pusherRight = new SparkMax(Constants.climberConstants.climberCANID, MotorType.kBrushless);
  public static SparkMax pusherLeft = new SparkMax(Constants.climberConstants.climber2CANID, MotorType.kBrushless);
  private static SparkMaxConfig rightConfig = new SparkMaxConfig();
  private static SparkMaxConfig leftConfig = new SparkMaxConfig();

  private static final double upSoftRotationLimit = Math.toRadians(45);
  private static final double downSoftRotationLimit = Math.toRadians(0); 
  private MotorJointIO climberIO = new MotorJointSparkMax(pusherRight, "Climber", Constants.climberConstants.climberCANID, 
                                                          downSoftRotationLimit, upSoftRotationLimit);
  private static MotorJointIOInputsAutoLogged climberInputs;

  public Climber() {

    this.climberInputs = new MotorJointIOInputsAutoLogged();

    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(false);
    rightConfig.smartCurrentLimit(10);
    pusherRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Follower will mirror the movements of the rightPusher
    leftConfig.follow(Constants.climberConstants.climberCANID,true);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(10);
    pusherLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climberClimb(boolean go) {
    if (go && !climberInputs.upperSoftLimitHit) {pusherRight.set(.1);} 
    else {pusherRight.set(0);}
  }

  // TODO can it reset? Ratchet??
  /*public void climberReset(boolean go) {
    if (go && !climberInputs.lowerSoftLimitHit) {
      pusherRight.set(-0.1);
    }
    else {
      pusherRight.set(0.0);
    }
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberIO.updateInputs(climberInputs);

    // Log summary data
    Logger.recordOutput(
        "Climber/connected", climberInputs.connected);
    Logger.recordOutput(
        "Climber/Measurement/absolutionPosition", climberInputs.absolutePosition);
    Logger.recordOutput(
        "Climber/Measurement/externalPosition", climberInputs.externalPosition);
    Logger.recordOutput(
        "Climber/Measurement/lowerSoftLimitHit", climberInputs.lowerSoftLimitHit);
    Logger.recordOutput(
        "Climber/Measurement/lowerSoftLimitHit", climberInputs.upperSoftLimitHit);
    Logger.recordOutput(
        "Climber/SetPoint/position", climberInputs.positionSetPoint);

  }
}
