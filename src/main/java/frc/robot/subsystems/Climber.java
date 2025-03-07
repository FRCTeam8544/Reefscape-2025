// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.LogUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  private static final double upSoftRotationLimit = 0; // rotations
  private static final double downSoftRotationLimit = 0.6; // TODO find real values....
  private static final int motorStallAmpLimit = 10;

  private static final double climberForwardMaxSpeed = 0.2;
  private static final double climberReverseMaxSpeed = 0.4;

  /** Creates a new Climber. */
  private SparkMax pusherRight = new SparkMax(Constants.climberConstants.climberCANID, MotorType.kBrushless);
  private SparkMax pusherLeft = new SparkMax(Constants.climberConstants.climber2CANID, MotorType.kBrushless);
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private AbsoluteEncoder encoder = pusherRight.getAbsoluteEncoder();

  // Joint definition and logging
  private MotorJointIO climberIO = new MotorJointSparkMax(pusherRight,"Climber",
                                                          Constants.climberConstants.climberCANID, 
                                                          downSoftRotationLimit, upSoftRotationLimit);
  private MotorJointIOInputsAutoLogged  climberInOutData = new MotorJointIOInputsAutoLogged();

  private boolean upStopLimitHit = false;
  private boolean downStopLimitHit = false;
  
  public Climber() {
    
    // Configure right motor
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(false);
    rightConfig.smartCurrentLimit(motorStallAmpLimit);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Enable hard limits with directly atatched limit switches
    rightConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    // Apply right motor configuration
    pusherRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Follower will mirror the movements of the rightPusher
    leftConfig.follow(Constants.climberConstants.climberCANID,true);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(motorStallAmpLimit);

    // Apply right motor configuration
    pusherLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climberClimb(boolean go) {
    if (go && !climberInOutData.upperSoftLimitHit && !climberInOutData.upperLimitHit) {
      setVelocitySetPoint(climberForwardMaxSpeed);
    } 
    else {
      setVelocitySetPoint(0);
    }
  }

  public void climberBack(boolean move){
    if(move && !climberInOutData.lowerSoftLimitHit && !climberInOutData.lowerLimitHit) { 
      setVelocitySetPoint(climberReverseMaxSpeed);
    }
    else {
      setVelocitySetPoint(0.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberIO.updateInputs(climberInOutData);

    // Check limits
    downStopLimitHit = climberInOutData.lowerLimitHit || climberInOutData.lowerSoftLimitHit;
    upStopLimitHit = climberInOutData.upperLimitHit || climberInOutData.upperSoftLimitHit;

    // Apply limits (Postive velocity is assumed to be climber deploy)
    if (encoder.getVelocity() > 0.0 && downStopLimitHit) {
      setVelocitySetPoint(0);
    }
    // (Negative velocity is assumed to be climber retract)
    else if (encoder.getVelocity() < 0.0 && upStopLimitHit) {
      setVelocitySetPoint(0);
    }
    
    // Log summary data
    LogUtil.logData("Climber", climberInOutData);
  }

  private void setVelocitySetPoint(double speed) {
    double limitedSpeed = 0.0;
    if (speed > 0.0) {
      climberInOutData.velocitySetPoint = Math.min(speed, climberForwardMaxSpeed);
    }
    else {
      climberInOutData.velocitySetPoint = Math.max(speed, climberReverseMaxSpeed);
    }
    pusherRight.set(limitedSpeed);
  }
  
  private void setPositionSetPoint(double position) {
    climberInOutData.velocitySetPoint = 0.0;
    climberInOutData.positionSetPoint = 0.0;
  }

}
