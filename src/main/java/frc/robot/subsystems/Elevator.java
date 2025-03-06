
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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
  private static RelativeEncoder encoder = leftMotorController.getExternalEncoder();

  /* Create elbow joint */
  private static SparkFlex elbowController = new SparkFlex(Constants.elevatorConstants.rightElbowCANID, MotorType.kBrushless);
  private static AbsoluteEncoder elbowEncoder = elbowController.getAbsoluteEncoder();

  private static SparkFlexConfig motorConfig = new SparkFlexConfig();
  private static SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
  private static SparkFlexConfig spinConfig = new SparkFlexConfig();

  // 5.53 inches per rotation of elevator output shaft
  final static double upSoftStopValue = 9.5; // Rotations, enough to reach level 4 coral
  final static double downSoftStopValue = 0;
  final static double backwardSoftStopValue = 0; // TODO need to set zero point in stow with rev client
  final static double forwardSoftStopValue = 0.3; // 108 degrees as rotations, TODO confirm this limit
  final static double altBackwardSoftStopValue = 0.05;
  final static double altForwardSoftStopValue = forwardSoftStopValue;
  final static double elbowAlternateLimitsElevatorThresh = 6; // Above this there is risk of elevator collision, leaving slack to allow elbow push out time
  
  private static MotorJointIOInputs elevatorInOutData;
  // Encoder is on the  robot left motor!!!
  private static MotorJointIO elevatorMotorIO = new MotorJointSparkFlex(leftMotorController, "Elevator", Constants.elevatorConstants.rightElevatorCANID, 
                                                                    downSoftStopValue, upSoftStopValue, false);
                          
  private static MotorJointIOInputs elbowInOutData;
  private static MotorJointIO elbowMotorIO = new MotorJointSparkFlex(elbowController, "Elbow", Constants.elevatorConstants.rightElbowCANID,
                                                                     backwardSoftStopValue, forwardSoftStopValue, true);

  /*private static DigitalInput forwardLimit = 
    new DigitalInput(Constants.elevatorConstants.forwardSwitchPort);
    private static DigitalInput backwardLimit =
    new DigitalInput(Constants.elevatorConstants.backwardSwitchPort);
  */

  private static DigitalInput upLimit = new DigitalInput(Constants.elevatorConstants.limitSwitchPort); // limit switches
  private static DigitalInput downLimit = new DigitalInput(Constants.elevatorConstants.limitSwitch2Port);

  public static boolean elevatorCalibrated = false;
  public static boolean forwardStopHit;
  public static boolean backwardStopHit; // These should not be static...
  public static boolean upStopHit;
  public static boolean downStopHit;
      
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

        elevatorCalibrated = false;

        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(10);
        motorConfig.inverted(true); // Clockwise is UP, nominal motor spin is counterclockwise
        motorController.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.smartCurrentLimit(10);
        leftMotorConfig.follow(Constants.elevatorConstants.rightElevatorCANID, true); 
        leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        leftMotorController.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setupElbowConfig();
      }
    
      public void setupElbowConfig() {
        spinConfig.idleMode(IdleMode.kBrake);
        spinConfig.inverted(false);
        spinConfig.smartCurrentLimit(10);
        elbowController.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

      @Override
      public void periodic() {
        // This method will be called once per scheduler run

        // Zero the relative external encoder, when the elevator touches the bottom limit switch
        if (downStop.getAsBoolean()) {
           elevatorMotorIO.setZeroOffset( encoder.getPosition() );
           elevatorCalibrated = true;
        }

        elbowMotorIO.updateInputs(elbowInOutData);
        elevatorMotorIO.updateInputs(elevatorInOutData);
        
        // This must be done after updateInputs so that the external position is accurate
        if (elevatorInOutData.externalPosition >= elbowAlternateLimitsElevatorThresh)
        {
          elbowMotorIO.setAlternateLimits(altBackwardSoftStopValue,altForwardSoftStopValue);
        }
        else {
          elbowMotorIO.clearAlternateLimits();
        }
        
        //forwardStopHit = forwardStop.getAsBoolean() || elbowInOutData.upperSoftLimitHit;
        //backwardStopHit = backwardStop.getAsBoolean() || elbowInOutData.lowerSoftLimitHit;
        forwardStopHit = elbowInOutData.upperSoftLimitHit;
        backwardStopHit = elbowInOutData.lowerSoftLimitHit;
    
        // Hard limits
        // Combine Hard limit wrist check with soft limit results:
        // TODO hook limit switches to the spark controllers directly???
        // What about feedback to the software to let it know that a limit has been reached?
        if (elevatorCalibrated) {
          upStopHit = upStop.getAsBoolean() || elevatorInOutData.upperSoftLimitHit;
          downStopHit = downStop.getAsBoolean() || elevatorInOutData.lowerSoftLimitHit;
        }
        else { // Ignore soft limits when elevator position is not calibrated
          upStopHit = upStop.getAsBoolean();
          downStopHit = downStop.getAsBoolean();
        }
    
        if (upStopHit || downStopHit) {
          motorController.set(0.0); // Stop imediately regardless of the running command
        }

        if (forwardStopHit || backwardStopHit) {
          elbowController.set(0.0); // Stop imediataly
        }
    
        LogUtil.logData(elbowMotorIO.getName(), elbowInOutData);
        LogUtil.logData(elevatorMotorIO.getName(), elevatorInOutData);
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
        {elbowController.set(.15);} //basically become number x (probs not 4)
       else{elbowController.set(0);}
      }
  
      //elbow basics
        public static void spinElbowForward(boolean go) {
          if (go && !forwardStopHit) {elbowController.set(.10);} 
        else {elbowController.set(0);}
      }
    
      public static void spinElbowBackwards(boolean execute) {
        if (execute && !backwardStopHit) {elbowController.set(-.10);} 
        else {elbowController.set(0);}
      }
    
      // Home the elevator to the down stop to determine the "zero" value
      // of the current relative encoder value.
      public static void calibrateElevatorEncoder() {
        if (!downStopHit) {
          motorController.set(-.05);
          elevatorCalibrated = false;
        }
        else {
          elevatorCalibrated = true;
          motorController.set(0);
        }
      }

      //problem range elevator/elbow
      public static void elevatorElbowIssueUp(){ //example transition range 3-5 find real one day

        final double elevatorPosition = elevatorInOutData.externalPosition;
        final double elbowPosition = elbowInOutData.absolutePosition;
 
        if (!elevatorCalibrated) {
          return;
        }

        if (upStopHit) {
           motorController.set(0.0);
        }
        else {
           motorController.set(0.15);
        }

        if (elevatorPosition > elbowAlternateLimitsElevatorThresh) {
          if (elbowEncoder.getPosition() < 0.0) {
            elbowController.set(0.10); // kick elbow out to zero position
          }
          else {
            elbowController.set(0);
          }
        }
      }

      public static void elevatorElbowIssueDown(){
        final double elevatorPosition = elevatorInOutData.externalPosition;
        final double elbowPosition = elbowInOutData.absolutePosition;
 
        if (!elevatorCalibrated) {
          return;
        }

        if (downStopHit) {
           motorController.set(0.0);
        }
        else {
           motorController.set(-0.15);
        }

        if (elevatorPosition > elbowAlternateLimitsElevatorThresh) {
          if (elbowEncoder.getPosition() >= 0.0) {
            elbowController.set(-0.10); // kick elbow out to zero position
          }
          else {
            elbowController.set(0);
          }
        }
      }

   public void updateDashboard(){
    SmartDashboard.putNumber("elevator Speed", encoder.getVelocity());  
    SmartDashboard.putBoolean("up limit hit", upStopHit);
    SmartDashboard.putBoolean("down limit hit", downStopHit);
    SmartDashboard.putBoolean("forward limit hit", forwardStopHit);
    SmartDashboard.putBoolean("backward limit hit", backwardStopHit);
  }

  public double getElevatorVelocity(){
    return encoder.getVelocity();
  }
}