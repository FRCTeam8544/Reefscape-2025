// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.subsystems.MotorJointSparkFlex;
import frc.robot.util.LogUtil;
import frc.robot.subsystems.MotorJointIO.MotorJointIOInputs;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private static SparkFlex motorController = new SparkFlex(Constants.elevatorConstants.rightElevatorCANID, MotorType.kBrushless);
  private static SparkFlex leftMotorController = new SparkFlex(Constants.elevatorConstants.leftElevatorCANID, MotorType.kBrushless);
  private static SparkFlex spinMotorRight = new SparkFlex(Constants.elevatorConstants.rightElbowCANID, MotorType.kBrushless);
  private static AbsoluteEncoder encoder = leftMotorController.getAbsoluteEncoder();
  private static AbsoluteEncoder elbowEncoder = spinMotorRight.getAbsoluteEncoder();

  private static SparkFlexConfig motorConfig = new SparkFlexConfig();
  private static SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
  private static SparkFlexConfig spinConfig = new SparkFlexConfig();
  
  final static double upSoftStopValue = Math.toRadians(0);
  final static double downSoftStopValue = Math.toRadians(90);
  final static double backwardSoftStopValue = Math.toRadians(-10);
  final static double forwardSoftStopValue = Math.toRadians(20);
  private MotorJointIOInputs elevatorInOutData;
  // Encoder is on the  robot left motor!!!
  private static MotorJointIO elevatorMotorIO = new MotorJointSparkFlex(leftMotorController, "Elevator", Constants.elevatorConstants.rightElevatorCANID, 
                                                                    downSoftStopValue, upSoftStopValue, false);
                          
  private MotorJointIOInputs elbowInOutData;
  private static MotorJointIO elbowMotorIO = new MotorJointSparkFlex(spinMotorRight, "Elbow", Constants.elevatorConstants.rightElbowCANID,
                                                                     backwardSoftStopValue, forwardSoftStopValue, false);

  /*private static DigitalInput forwardLimit = 
    new DigitalInput(Constants.elevatorConstants.forwardSwitchPort);
    private static DigitalInput backwardLimit =
    new DigitalInput(Constants.elevatorConstants.backwardSwitchPort);
  */

  private static DigitalInput upLimit = new DigitalInput(Constants.elevatorConstants.limitSwitchPort); // limit switches
  private static DigitalInput downLimit = new DigitalInput(Constants.elevatorConstants.limitSwitch2Port);

  public static boolean forwardStopHit;
  public static boolean backwardStopHit; // These should not be static...
  public static boolean upStopHit;
  public  static boolean downStopHit;
      
  public BooleanSupplier upStop =
            () -> {
              return upLimit.get();
            };
        public BooleanSupplier downStop =
            () -> {
              return downLimit.get();
            };
    
      public Elevator() {
    
        elevatorInOutData = new MotorJointIOInputsAutoLogged();
        elbowInOutData = new MotorJointIOInputsAutoLogged();
    
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(20);
        motorConfig.inverted(false);
        motorController.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.smartCurrentLimit(20);
        leftMotorConfig.follow(Constants.elevatorConstants.rightElevatorCANID, true); 
        leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        leftMotorController.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setupElbowConfig();
      }
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run

        elbowMotorIO.updateInputs(elbowInOutData);
        elevatorMotorIO.updateInputs(elevatorInOutData);
       
        // Combine Hard limit wrist check with soft limit results:
        // TODO hook limit switches to the spark controllers directly???
        // What about feedback to the software to let it know that a limit has been reached?
        upStopHit = upStop.getAsBoolean() || elevatorInOutData.upperSoftLimitHit;
    
        downStopHit = downStop.getAsBoolean() || elevatorInOutData.lowerSoftLimitHit;
    
        //forwardStopHit = forwardStop.getAsBoolean() || elbowInOutData.upperSoftLimitHit;
        //backwardStopHit = backwardStop.getAsBoolean() || elbowInOutData.lowerSoftLimitHit;
        forwardStopHit = elbowInOutData.upperSoftLimitHit;
        backwardStopHit = elbowInOutData.lowerSoftLimitHit;
    
        // Hard limits
        // Combine Hard limit wrist check with soft limit results:
        // TODO hook limit switches to the spark controllers directly???
        // What about feedback to the software to let it know that a limit has been reached?
        upStopHit = upStop.getAsBoolean() || elevatorInOutData.upperSoftLimitHit;
        downStopHit = downStop.getAsBoolean() || elevatorInOutData.lowerSoftLimitHit;
    
        //if (upStopHit || downStopHit) {
        //  motorController.set(0.0); // Stop imediately regardless of the running command
        //}
    
        LogUtil.logData(elbowMotorIO.getName(), elbowInOutData);
    
        LogUtil.logData(elevatorMotorIO.getName(), elevatorInOutData);
      }
    
      public void setupElbowConfig() {
        spinConfig.idleMode(IdleMode.kBrake);
        spinConfig.inverted(false);
        spinConfig.smartCurrentLimit(40);
        spinMotorRight.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

      //elbow auto pose 
      public void elbowAutoScore(boolean m) {
        if(m && elbowEncoder.getPosition() > 4 || elbowEncoder.getPosition() < 4) 
        {spinMotorRight.set(.15);} //basically become number x (probs not 4)
       else{spinMotorRight.set(0);}
      }
  
      //elbow basics
        public static void spinElbowForward(boolean go) {
          if (go && !forwardStopHit) {spinMotorRight.set(.10);} 
        else {spinMotorRight.set(0);}
      }
    
      public static void spinElbowBackwards(boolean execute) {
        if (execute && !backwardStopHit) {spinMotorRight.set(-.10);} 
        else {spinMotorRight.set(0);}
      }
    
     //problem range elevator/elbow
      public static void elevatorElbowIssueUp(){ //example transition range 3-5 find real one day
        if(!upStopHit && encoder.getPosition() < 3 || encoder.getPosition() > 5)
        {motorController.set(.15);}

        if (encoder.getPosition() <=5 && encoder.getPosition() >=3){
          motorController.set(.15);
          if(elbowEncoder.getPosition() <= 0){
            spinMotorRight.set(.1);}
          else{spinMotorRight.set(0);}}}

      public static void elevatorElbowIssueDown(){
        if(!downStopHit && encoder.getPosition() < 3 || encoder.getPosition() > 5)
        {motorController.set(-.15);}

        if(encoder.getPosition() <= 5 && encoder.getPosition() >= 3){
          motorController.set(-.15);
          if(elbowEncoder.getPosition() >= 0){
            spinMotorRight.set(-.1);}
          else{spinMotorRight.set(0);
          }
        }
      }

   public void updateDashboard(){
    SmartDashboard.putNumber("elevator Speed", encoder.getVelocity());  
    SmartDashboard.putBoolean("up limit hit", upStopHit);
    SmartDashboard.putBoolean("down limit hit", downStopHit);
  }

  public double getElevatorVelocity(){
    return encoder.getVelocity();
  }
}
