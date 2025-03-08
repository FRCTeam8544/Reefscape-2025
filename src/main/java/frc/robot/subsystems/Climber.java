// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.LogUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  private static final double backwardSoftRotationLimit = 0; // rotations
  private static final double forwardSoftRotationLimit = 0.6; // TODO find real values....
  private static final int motorStallAmpLimit = 20;

  private static final double climberForwardMaxSpeed = 0.2;
  private static final double climberReverseMaxSpeed = 0.4;

  /** Creates a new Climber. */
  private SparkMax pusherRight = new SparkMax(Constants.climberConstants.climberCANID, MotorType.kBrushless);
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  //private AbsoluteEncoder encoder = pusherRight.getAbsoluteEncoder();
  private SparkClosedLoopController closedLoopController = pusherRight.getClosedLoopController();

  // Joint definition and logging
  private MotorJointIO climberIO = new MotorJointSparkMax(pusherRight,"Climber",
                                                          Constants.climberConstants.climberCANID, 
                                                          backwardSoftRotationLimit, forwardSoftRotationLimit);
  private MotorJointIOInputsAutoLogged  climberInOutData = new MotorJointIOInputsAutoLogged();

  private boolean forwardStopLimitHit = false;
  private boolean reverseStopLimitHit = false;

  public Climber() {
    
    // Configure right motor
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(false);
    rightConfig.smartCurrentLimit(motorStallAmpLimit);
    /*rightConfig.encoder.positionConversionFactor(1);
    rightConfig.encoder.velocityConversionFactor(1);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.climberConstants.climberKP, ClosedLoopSlot.kSlot0)
      .i(Constants.climberConstants.climberKI, ClosedLoopSlot.kSlot0)
      .d(Constants.climberConstants.climberKD, ClosedLoopSlot.kSlot0)
      .velocityFF(Constants.climberConstants.climberFF) // probably neo 550
      .outputRange(-1, 1);*/

    // Enable hard limits with directly atatched limit switches
    rightConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    // Set Smart Motion / Smart Velocity parameters
   // final int smartMotionSlot = 0;
   // closedLoopController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    //closedLoopController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
   // closedLoopController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    //closedLoopController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
       // .p(0.1)
      //  .i(0)
      //  .d(0)
      //  .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
      //  .p(0.0001, ClosedLoopSlot.kSlot1)
     //   .i(0, ClosedLoopSlot.kSlot1)
     //   .d(0, ClosedLoopSlot.kSlot1)
      //  .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
      //  .outputRange(-1, 1, ClosedLoopSlot.kSlot1);


    // Apply right motor configuration
    pusherRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climberClimb(boolean go) {
    if (go && !forwardStopLimitHit) {
      //setVelocitySetPoint(positionToClimbSpeed(encoder.getPosition()));
      setVelocitySetPoint(climberForwardMaxSpeed);
    } 
    else {
      setVelocitySetPoint(0);
    }
  }

  public void climberBack(boolean move){
    if(move && !reverseStopLimitHit) { 
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
    reverseStopLimitHit = climberInOutData.lowerLimitHit || climberInOutData.lowerSoftLimitHit;
    forwardStopLimitHit = climberInOutData.upperLimitHit || climberInOutData.upperSoftLimitHit;

    // Apply limits (Postive velocity is assumed to be climber deploy)
   // if (encoder.getVelocity() > 0.0 && downStopLimitHit) {
    //  setVelocitySetPoint(0);
   // }
    // (Negative velocity is assumed to be climber retract)
   // else if (encoder.getVelocity() < 0.0 && upStopLimitHit) {
   //   setVelocitySetPoint(0);
   // }
    
    // Log summary data
    LogUtil.logData("Climber", climberInOutData);
  }

  // Speed is percentage of max climber speed
  private void setVelocitySetPoint(double speed) {
    
    if (speed > 0.0) {
      climberInOutData.velocitySetPoint = Math.min(speed, climberForwardMaxSpeed);
    }
    else {
      climberInOutData.velocitySetPoint = Math.max(speed, climberReverseMaxSpeed);
    }

    double limitedSpeed = 0.0;
    final double climbRotationExtent = forwardSoftRotationLimit - backwardSoftRotationLimit;
    final double climbTimeSec = 5;
    final double maxClimberSpeed = Units.rotationsToRadians(climbRotationExtent) / climbTimeSec; // radians per second

    closedLoopController.setReference(Units.radiansPerSecondToRotationsPerMinute(limitedSpeed * maxClimberSpeed),
                                      ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
  
  private double positionToClimbSpeed(double position) {
    
    final double length = forwardSoftRotationLimit - backwardSoftRotationLimit;
    final double startToCoastPos = backwardSoftRotationLimit + Units.degreesToRotations(5);
    final double coastToPush = startToCoastPos + Units.degreesToRotations(90);
    final double pushToEnd = forwardSoftRotationLimit - coastToPush;
    
    // Region speeds in percent of max speed
    final double startSpeed = 0.3;
    final double coastSpeed = 1.0;
    final double pushSpeed = 0.2;
    
    final double positionRatio = (position - backwardSoftRotationLimit) / length; // 0 to 1 of climber rotation limit
    if (position < startToCoastPos) {
      return MathUtil.interpolate(startSpeed, coastSpeed, positionRatio);
    }
    else if ((position >= startToCoastPos) &&
             (position < coastToPush) ) {
      return coastSpeed;
    }
    else if ((position >= coastToPush) &&
            (position < pushToEnd)) {
      return MathUtil.interpolate(coastSpeed,pushSpeed,positionRatio);
    }
    else {
      return 0.0;
    }
  }
}
