// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.LogUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static SparkMax pusherRight = new SparkMax(Constants.climberConstants.climberCANID, MotorType.kBrushless);
  public static SparkMax pusherLeft = new SparkMax(Constants.climberConstants.climber2CANID, MotorType.kBrushless);
  private static SparkMaxConfig rightConfig = new SparkMaxConfig();
  private static SparkMaxConfig leftConfig = new SparkMaxConfig();
  private static AbsoluteEncoder encoder = pusherRight.getAbsoluteEncoder();

  private static final double upSoftRotationLimit = Math.toRadians(45);
  private static final double downSoftRotationLimit = Math.toRadians(0); 
  private MotorJointIO climberIO = new MotorJointSparkMax(pusherRight, "Climber", Constants.climberConstants.climberCANID, 
                                                          downSoftRotationLimit, upSoftRotationLimit);
  private static MotorJointIOInputsAutoLogged climberInOutData;

  public Climber() {

    this.climberInOutData = new MotorJointIOInputsAutoLogged();

    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(false);
    rightConfig.smartCurrentLimit(10);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pusherRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Follower will mirror the movements of the rightPusher
    leftConfig.follow(Constants.climberConstants.climberCANID,true);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(10);
    pusherLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climberClimb(boolean go) {
    if (go && !climberInOutData.upperSoftLimitHit && !climberInOutData.upperLimitHit) {
      pusherRight.set(.2);} 
    else {pusherRight.set(0);}
  }

  public void climberBack(boolean move){
    if(move){ 
      pusherRight.set(-.2);}
      else{pusherRight.set(0);}

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
    climberIO.updateInputs(climberInOutData);

    // Update commanded outputs
    // climberInputs.setPointVelocity ....

    // Log summary data
    LogUtil.logData("Climber", climberInOutData);
  }
}
