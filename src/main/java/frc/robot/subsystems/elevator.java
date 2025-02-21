// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.MotorJointSparkFlex;
import frc.robot.subsystems.MotorJointIO.MotorJointIOInputs;
import frc.robot.subsystems.MotorJointIOInputsAutoLogged;

public class elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private static SparkFlex motorController = new SparkFlex(Constants.elevatorConstants.rightElevatorCANID, MotorType.kBrushless);
  private static SparkFlex leftMotorController = new SparkFlex(Constants.elevatorConstants.leftElevatorCANID, MotorType.kBrushless);
  private static SparkFlex spinMotorRight = new SparkFlex(Constants.elevatorConstants.rightElbowCANID, MotorType.kBrushless);
  private static SparkFlex spinMotorLeft = new SparkFlex(Constants.elevatorConstants.leftElbowCANID, MotorType.kBrushless);

  private static SparkFlexConfig motorConfig = new SparkFlexConfig();
  private static SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
  private static SparkFlexConfig spinConfig = new SparkFlexConfig();
  private static SparkFlexConfig leftSpinConfig = new SparkFlexConfig();

  
  final static double upSoftStopValue = Math.toRadians(0);
  final static double downSoftStopValue = Math.toRadians(90);
  final static double backwardSoftStopValue = Math.toRadians(-10);
  final static double forwardSoftStopValue = Math.toRadians(20);
  private MotorJointIOInputs elevatorInputs;
  // Encoder is on the  robot left motor!!!
  private static MotorJointIO elevatorMotorIO = new MotorJointSparkFlex(leftMotorController, "Elevator", Constants.elevatorConstants.rightElevatorCANID, 
                                                                    downSoftStopValue, upSoftStopValue);
                          
  private MotorJointIOInputs elbowInputs;
  private static MotorJointIO elbowMotorIO = new MotorJointSparkFlex(spinMotorRight, "Elbow", Constants.elevatorConstants.rightElbowCANID,
                                                                     backwardSoftStopValue, forwardSoftStopValue);

  /*private static DigitalInput forwardLimit = 
      new DigitalInput(Constants.elevatorConstants.forwardSwitchPort);
  private static DigitalInput backwardLimit =
      new DigitalInput(Constants.elevatorConstants.backwardSwitchPort);
*/
  private static DigitalInput upLimit =
      new DigitalInput(Constants.elevatorConstants.limitSwitchPort); // limit switches
  private static DigitalInput downLimit =
      new DigitalInput(Constants.elevatorConstants.limitSwitch2Port);

  public static boolean forwardStopHit;
  public static boolean backwardStopHit; // These should not be static...
  public boolean upStopHit;
  public boolean downStopHit;

  public BooleanSupplier upStop =
      () -> {
        return upLimit.get();
      };
  public BooleanSupplier downStop =
      () -> {
        return downLimit.get();
      };

  public elevator() {

    elevatorInputs = new MotorJointIOInputsAutoLogged();
    elbowInputs = new MotorJointIOInputsAutoLogged();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(10);
    motorController.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorConfig.inverted(true);

    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.smartCurrentLimit(10);
    leftMotorConfig.follow(Constants.elevatorConstants.rightElevatorCANID, true);
    leftMotorController.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setupElbowConfig();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    elbowMotorIO.updateInputs(elbowInputs);
    elevatorMotorIO.updateInputs(elevatorInputs);
   
    // Combine Hard limit wrist check with soft limit results:
    // TODO hook limit switches to the spark controllers directly???
    // What about feedback to the software to let it know that a limit has been reached?
    upStopHit = upStop.getAsBoolean() || elevatorInputs.upperSoftLimitHit;

    downStopHit = downStop.getAsBoolean() || elevatorInputs.lowerSoftLimitHit;

    //forwardStopHit = forwardStop.getAsBoolean() || elbowInputs.upperSoftLimitHit;
    //backwardStopHit = backwardStop.getAsBoolean() || elbowInputs.lowerSoftLimitHit;
    forwardStopHit = false;//elbowInputs.upperSoftLimitHit;
    backwardStopHit = false;//elbowInputs.lowerSoftLimitHit;

  // Hard limits
    // Combine Hard limit wrist check with soft limit results:
    // TODO hook limit switches to the spark controllers directly???
    // What about feedback to the software to let it know that a limit has been reached?
    upStopHit = upStop.getAsBoolean(); //|| elevatorInputs.upperSoftLimitHit;
    downStopHit = downStop.getAsBoolean(); //|| elevatorInputs.lowerSoftLimitHit;

    //if (upStopHit || downStopHit) {
    //  motorController.set(0.0); // Stop imediately regardless of the running command
    //}

    // Log summary data
    Logger.recordOutput(
        "Elbow/connected", elbowInputs.connected);
    Logger.recordOutput(
        "Elbow/Measurement/absolutionPosition", elbowInputs.absolutePosition);
    Logger.recordOutput(
        "Elbow/Measurement/externalPosition", elbowInputs.externalPosition);
    Logger.recordOutput(
        "Elbow/Measurement/lowerSoftLimitHit", elbowInputs.lowerSoftLimitHit);
    Logger.recordOutput(
        "Elbow/Measurement/lowerSoftLimitHit", elbowInputs.upperSoftLimitHit);
    Logger.recordOutput(
        "Elbow/SetPoint/position", elbowInputs.positionSetPoint);

  // Log summary data
    Logger.recordOutput(
        "Elevator/connected", elevatorInputs.connected);
    Logger.recordOutput(
        "Elevator/Measurement/absolutionPosition", elevatorInputs.absolutePosition);
    Logger.recordOutput(
        "Elevator/Measurement/externalPosition", elevatorInputs.externalPosition);
    Logger.recordOutput(
        "Elevator/Measurement/lowerSoftLimitHit", elevatorInputs.lowerSoftLimitHit);
    Logger.recordOutput(
        "Elevator/Measurement/lowerSoftLimitHit", elevatorInputs.upperSoftLimitHit);
    Logger.recordOutput(
        "Elevator/SetPoint/position", elevatorInputs.positionSetPoint);
  }

  public void setupElbowConfig() {
    spinConfig.idleMode(IdleMode.kBrake);
    spinConfig.inverted(false);
    spinConfig.smartCurrentLimit(10);
    spinMotorRight.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    leftSpinConfig.idleMode(IdleMode.kBrake);
    leftSpinConfig.smartCurrentLimit(10);
    leftSpinConfig.follow(Constants.elevatorConstants.rightElbowCANID,true);
    spinMotorLeft.configure(leftSpinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  //elevator basic up/down
  public void elevatorMove(boolean up) {
    if (!upStopHit && up) {motorController.set(.15);}
    if (upStopHit || !up) {motorController.set(0);}
  }
  public void elevatorLow(boolean down) {
    if (!downStopHit && down) {motorController.set(-.15);}
    if (downStopHit || !down) {motorController.set(0);}
  }

//levels??? encoder???
 /*   public void level4() {
  if(!upStopHit || encoder.getPosition() > 10)
    {motorController.set(.1);}
    else {motorController.set(0);}}
*/
//elbow basics
  public static void spinElbowForward(boolean go) {
    if (go && !forwardStopHit) {spinMotorRight.set(.10);} 
    else {spinMotorRight.set(0);}
  }

  public static void spinElbowBackwards(boolean execute) {
    if (execute && !backwardStopHit) {spinMotorRight.set(-.10);} 
    else {spinMotorRight.set(0);}
  }

   public void updateDashboard(){
    SmartDashboard.putNumber("elevator Speed", motorController.getEncoder().getVelocity());  
  }

  public double getElevatorVelocity(){
    return motorController.getEncoder().getVelocity();
  }
}
