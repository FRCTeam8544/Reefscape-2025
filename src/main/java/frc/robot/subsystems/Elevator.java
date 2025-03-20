
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.MotorJointSparkFlex;
import frc.robot.util.LogUtil;
import frc.robot.subsystems.MotorJointIO.MotorJointIOInputs;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private static SparkFlex motorController = new SparkFlex(Constants.elevatorConstants.rightElevatorCANID, MotorType.kBrushless);
  private static SparkFlex leftMotorController = new SparkFlex(Constants.elevatorConstants.leftElevatorCANID, MotorType.kBrushless);
  public static RelativeEncoder encoder = leftMotorController.getExternalEncoder();
  private static SparkClosedLoopController closedLoop = leftMotorController.getClosedLoopController();
  private static SparkFlexConfig motorConfig = new SparkFlexConfig();
  private static SparkFlexConfig leftMotorConfig = new SparkFlexConfig();

  /* Create elbow joint */
  private static SparkFlex elbowController = new SparkFlex(Constants.elevatorConstants.rightElbowCANID, MotorType.kBrushless);
  public static AbsoluteEncoder elbowEncoder = elbowController.getAbsoluteEncoder();
  private static SparkClosedLoopController elbowClosedLoop = elbowController.getClosedLoopController();
  private static SparkFlexConfig spinConfig = new SparkFlexConfig();
  
  // 5.53 inches per rotation of elevator output shaft
  final static int stallLimit = 40; //change to change both at once!
  final static double elevatorMaxSpeed = .2; // % of max speed ???
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
  
  private double DEFAULT_ELEVATOR_KG = 0;
  private double elevator_kG = DEFAULT_ELEVATOR_KG; // Tunable to determine the needed kG term to resist gravity

  private SparkLimitSwitch upLimit = leftMotorController.getForwardLimitSwitch();
  private SparkLimitSwitch downLimit = leftMotorController.getReverseLimitSwitch();

  public boolean elevatorCalibrated = false;
  public boolean forwardStopHit;
  public boolean backwardStopHit; // These should not be static...
  public boolean upStopHit;
  public boolean downStopHit;

/* */      
  public BooleanSupplier upStop =
            () -> {
              return upLimit.isPressed();
            };
        public BooleanSupplier downStop =
            () -> {
              return downLimit.isPressed();
            };
    
      public Elevator() {

        //left motor is now leader not right
    
        elevatorInOutData = new MotorJointIOInputsAutoLogged();
        elbowInOutData = new MotorJointIOInputsAutoLogged();

        elevatorCalibrated = false;
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(stallLimit);
        motorConfig.follow(Constants.elevatorConstants.leftElevatorCANID, true); 
        motorConfig.voltageCompensation(12);
        motorConfig.softLimit.forwardSoftLimitEnabled(false);
        motorConfig.softLimit.reverseSoftLimitEnabled(false);
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(false);
        motorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        motorController.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.smartCurrentLimit(stallLimit);
        leftMotorConfig.inverted(false);
        leftMotorConfig.voltageCompensation(12); 
        leftMotorConfig.softLimit.forwardSoftLimitEnabled(true);
        leftMotorConfig.softLimit.forwardSoftLimit(upSoftStopValue);
        leftMotorConfig.softLimit.reverseSoftLimitEnabled(true);
        leftMotorConfig.softLimit.reverseSoftLimit(downSoftStopValue);
        leftMotorConfig.limitSwitch
          .forwardLimitSwitchType(Type.kNormallyOpen)
          .forwardLimitSwitchEnabled(true)
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen); 
        leftMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          // Position control (untuned)
          .p(0.0005, ClosedLoopSlot.kSlot0)
          .i(0, ClosedLoopSlot.kSlot0)
          .d(0.00000, ClosedLoopSlot.kSlot0)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
          // Velocity control
          .p(0.0006, ClosedLoopSlot.kSlot1)
          .i(0.00001, ClosedLoopSlot.kSlot1)
          .d(0.0006, ClosedLoopSlot.kSlot1)
          .iMaxAccum(0.018, ClosedLoopSlot.kSlot1) // Prevent integral runaway!!!!
          .outputRange(-1,1, ClosedLoopSlot.kSlot1)
          .velocityFF(1/Constants.NeoVortexConstants.motorKV, ClosedLoopSlot.kSlot1); //only used in velocity loop & set based on motor type
        leftMotorController.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        setupElbowConfig();
      }
    
      public void setupElbowConfig() {
        //final double kMaxElbowVelocityRPM = 
        spinConfig.idleMode(IdleMode.kBrake);
        spinConfig.inverted(false);
        spinConfig.voltageCompensation(12); 
        spinConfig.smartCurrentLimit(40);
        spinConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .p(0.000000005, ClosedLoopSlot.kSlot0)
          .i(0, ClosedLoopSlot.kSlot0)
          .d(0.00000, ClosedLoopSlot.kSlot0)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
       //   .maxMotion.maxVelocity(kMaxWristVelocityRPM, ClosedLoopSlot.kSlot0)
        //            .maxAcceleration(kMaxWristAcceleration, ClosedLoopSlot.kSlot0);
        elbowController.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }

      @Override
      public void periodic() {
        // This method will be called once per scheduler run

        // Zero the relative external encoder, when the elevator touches the bottom limit switch
        if (downStop.getAsBoolean()) {
           elevatorMotorIO.setZeroOffset(encoder.getPosition());
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
        
        upStopHit = false;
        downStopHit = false;

        //forwardStopHit = forwardStop.getAsBoolean() || elbowInOutData.upperSoftLimitHit;
        //backwardStopHit = backwardStop.getAsBoolean() || elbowInOutData.lowerSoftLimitHit;
        //forwardStopHit = elbowInOutData.upperSoftLimitHit;
        //backwardStopHit = elbowInOutData.lowerSoftLimitHit;
    
        // Hard limits
        // Combine Hard limit wrist check with soft limit results:
        // TODO hook limit switches to the spark controllers directly???
        // What about feedback to the software to let it know that a limit has been reached?

       /* if (elevatorCalibrated) {
          upStopHit = false; //|| elevatorInOutData.upperSoftLimitHit; //took out limit switch
          downStopHit = false; //|| elevatorInOutData.lowerSoftLimitHit;
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
        } */

    
        LogUtil.logData(elbowMotorIO.getName(), elbowInOutData);
        LogUtil.logData(elevatorMotorIO.getName(), elevatorInOutData);

        updateDashboard();

      }
      
      // Drive the motor at a specific velocity setPoint ( setPoint / maxSpeed )
      // Note max speed may vary as the elevator nears the extents of its travel.
      private void setVelocitySetPoint(double setPoint){
        //final double maxSpeedRatio = 1;//getMaxElevatorSpeedPercent();

        // Limit speed percentage within range and preserve direction
        //double adjSetPoint = Math.copySign(Math.min(Math.abs(setPoint), maxSpeedRatio),setPoint);

        final double maxSpeed = getMaxElevatorSpeedRadiansPerSec();
        double commandedSpeedInRPM = setPoint * Units.radiansPerSecondToRotationsPerMinute(maxSpeed);

        elevatorInOutData.velocitySetPoint = commandedSpeedInRPM;
        elevatorInOutData.positionSetPoint = 0;
        elevatorInOutData.voltageSetPoint = 0;
        closedLoop.setReference(commandedSpeedInRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      }

      // Attempt to hit a specific elevator position, via PID
      private void setPositionSetPoint(double pointSet){
        pointSet = encoder.getPosition();

        elevatorInOutData.positionSetPoint = pointSet;
        elevatorInOutData.velocitySetPoint = 0;
        elevatorInOutData.voltageSetPoint = 0;

        closedLoop.setReference(pointSet, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }

      // Apply a voltage to move the elevator motors, without PID control
      private void setVoltageSetPoint(double voltage) {
        elevatorInOutData.voltageSetPoint = voltage;
        leftMotorController.setVoltage(voltage);
      }

      // Attempt to keep the elevator at the same position
      public void holdPositionSetPoint() {
        // External position is used here because it already adjusted for zero offset and
        // the same position will be used as a set point for 20 ms.
        // Use max of zero and external position to prevent out of bounds position holding
        //double limitedPosition = Math.min(elevatorInOutData.externalPosition,upSoftStopValue);
        //setPositionSetPoint(0);
        setVelocitySetPoint(0.0);
       // setVoltageSetPoint(elevator_kG);
      }
      
      // Moves elevator to the specified position, in revolutions from zero point, must be positive
      public void runElevatorToPosition(double position)
      {
      // TODO need to finalize position PID loop tuning and add velocity profile limits
      // For now cheese this with velocity run until position, probably will overshoot
        final double limitedPosition = Math.max( Math.min(upSoftStopValue, position), // enforce upper limit
                                                 downSoftStopValue // enforce lower limit
                                               );
        final double positionDelta = Math.abs(position - elevatorInOutData.externalPosition);

       // figure out max distance... or just use smart motion limits???
       // getMaxElevatorSpeedRadiansPerSec()

       // runElevatorVelocity(Math.copySign(velocity, positionDelta));
      }

      // Run the elevator by commanding a speed as percent of max elevator speed
      public void runElevatorVelocity(double speed) {
        if (speed >= 0) {
          if (!upStopHit) {
            setVelocitySetPoint(speed);
          }
          else {
            holdPositionSetPoint();
          }
        }
        else {
          if (!downStopHit) {
            setVelocitySetPoint(speed);
          }
          else {
            holdPositionSetPoint();
          }
        }
      }

      //elevator basic up/down
      public void elevatorMove(boolean up) {
        if (!upStopHit && up) {
          setVelocitySetPoint(0.4);
          //setVoltageSetPoint(elevator_kG); 
          //elevator_kG += .0001;
        }
        else {
          holdPositionSetPoint();
         //setVoltageSetPoint(0);
        }

        // TODO hijack elbow forward to increase kG every 1/4 second the button is held
      //  if (up && (updateCounter % 25 == 0)) {
      //    elevator_kG += 0.05;
      //  }
      }

      public void elevatorLow(boolean down) {

      // TODO hijack elbow forward to decrease kG every 1/4 second the button is held
       // if (down && (updateCounter % 25 == 0)) {
      //    elevator_kG -= 0.05;
       // }

       /* if (elevatorCalibrated && down && (elevatorInOutData.externalPosition <= 0.5)) { // last couple inches
          // Do nothing, let it brake mode fall to the zero position
        }*/
          {if (!downStopHit && down) {
            setVelocitySetPoint(-0.3);
            //setPositionSetPoint(encoder.getPosition());
            //setVoltageSetPoint(elevator_kG);
            //elevator_kG -= .0001;
          }
          else {
          holdPositionSetPoint();
          //setVoltageSetPoint(0);
          }
        }
      }

      public void turnElbowToPosition(double position) {
        if ((position >= backwardSoftStopValue) ||
            (position <= forwardSoftStopValue)) {
          elbowClosedLoop.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
      }
  
      //elbow basics
      public void spinElbowForward(boolean go) {
         if (go && !forwardStopHit) {elbowController.set(.2);} 
        else {elbowController.set(0);}
      }
    
      public void spinElbowBackwards(boolean execute) {
        if (execute && !backwardStopHit) {elbowController.set(-.2);} 
        else {elbowController.set(0);}
      }
    

      //problem range elevator/elbow
      public void elevatorElbowIssueUp(){ //example transition range 3-5 find real one day

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

      public void elevatorElbowIssueDown(){
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
    SmartDashboard.putNumber("elevatorKgTerm", elevator_kG);
  }

  public void logPose(String prefix, int snapshotId) {
      Logger.recordOutput(prefix + "/Id", snapshotId);
      Logger.recordOutput(
          prefix + "/Elevator/externalPosition", elevatorInOutData.externalPosition);
      Logger.recordOutput(
          prefix + "/Elevator/zeroOffset", elevatorInOutData.zeroOffset);

      Logger.recordOutput(
          prefix + "/Elbow/absolutePosition", elbowInOutData.absolutePosition);
  }

  // Retrieve elevator height
  public double getElevatorPosition() {
    final double rotationsToInches = 5.53;
    // Todo add height from floor to start of elevator?
    return rotationsToInches * elevatorInOutData.externalPosition; // return position in inches
  }

  public double getElevatorVelocity(){
    return encoder.getVelocity();
  }
    
  // Provide the max elevator speed in radians per second
  private double getMaxElevatorSpeedRadiansPerSec() {
    // Radians per second
    final double timeSec = .2;
    final double maxSpeedRadiansPerSecond = 2 * (Math.PI) / timeSec;
    //return getMaxElevatorSpeedPercent() * maxSpeedRadiansPerSecond;
    return maxSpeedRadiansPerSecond;
  }
  
 /*  private double getMaxElevatorSpeedPercent() {
    // Ratio of speed vs max possible speed e.g. 1 is full speed, 0.5 is half max speed
    double elevatorSpeedFactor = 1;
    
    final double elePos = elevatorInOutData.externalPosition;

    // TODO clean this logic up....
    // The goal here is to define speed limits near the ends of the elevator travel
    if (elePos < 1) {
      double segmentLength = 1 - 0; // Must match current and prior if boundary values
      elevatorSpeedFactor = MathUtil.interpolate(0, .2, elePos / segmentLength);
    }
    else if (elePos < 2) {
      double segmentLength = 2 - 1; // Must match current and prior if boundary values
      elevatorSpeedFactor = MathUtil.interpolate(0.2, .4, (elePos - 1) / segmentLength );
    }
    else if (elePos < 2.5) {
      double segmentLength = 2.5 - 2; // Must match current and prior if boundary values
      elevatorSpeedFactor = MathUtil.interpolate(0.4, .6, (elePos - 2.5) / segmentLength );
    }
    else if (elePos < 7.5) {
      // todo LIMIT ACCELLERATION....
      elevatorSpeedFactor = 1;
    }
    else if (elePos < 8.5) {
      double segmentLength = 8.5 - 7.5;
      elevatorSpeedFactor = MathUtil.interpolate(0.6,0.4, elePos - 8.5 / segmentLength);
    }
    else if (elePos < 9.2) {
      double segmentLength = 9.2 - 8.5;
      elevatorSpeedFactor = MathUtil.interpolate(0.4,0.1, elePos - 9.2 / segmentLength);
    }
    else {
      elevatorSpeedFactor = 0.05;
    }

    //return elevatorSpeedFactor;
    return 1;
  }
*/
}