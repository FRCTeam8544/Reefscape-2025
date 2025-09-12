// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import au.grapplerobotics.LaserCan;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.GameConstants;
import frc.robot.subsystems.LaserCAN;
import frc.robot.subsystems.LaserCANIO;
import frc.robot.subsystems.LaserCANIO.LaserCANIOInputs;
import frc.robot.subsystems.LaserCANIOInputsAutoLogged;
import frc.robot.subsystems.MotorJointSparkFlex;
import frc.robot.subsystems.MotorJointIO.MotorJointIOInputs;
import frc.robot.subsystems.MotorJointIOInputsAutoLogged;

import frc.robot.util.LogUtil;
import frc.robot.util.SegmentedMapping;

public class ClawIntake extends SubsystemBase {
  
  private final double intakeWidth_mm = 250;
  private final int coralAcquiredCountThreshold = 10; // Wait N * 20 ms before declaring coral acquired

  private boolean coralAcquired;
  private int coralPresentCount;

  private static LaserCANIOInputsAutoLogged coralLaserInputs;
  private static LaserCANIO seabassIO = new LaserCAN("CoralLaser",
                                             Constants.clawIntakeConstants.laser1CANID,
                                             LaserCAN.FieldOfView.NARROW_4_BY_4);

  /** Creates a new ClawIntake. */
  public static SparkMax rollerRight = new SparkMax(Constants.clawIntakeConstants.rollerCANID, MotorType.kBrushed);
  private static SparkMax rollerLeft = new SparkMax(Constants.clawIntakeConstants.roller2CANID, MotorType.kBrushed);
  
  private final double upSoftRotationLimit = .095; // Rotations
  private final double downSoftRotationLimit = -.095;  // Rotations
  
  private MotorJointIOInputs wristInOutData;
  private static SparkFlex wrist = new SparkFlex(Constants.clawIntakeConstants.wristCANID, MotorType.kBrushless);
  private static SparkClosedLoopController wristController = wrist.getClosedLoopController();
  private final double kMaxWristVelocityRPM = (0.2/1) * 60; // (rotation distance / second ) * sec/min Convert rotations per second to RPM
  private final double kMaxWristAcceleration = kMaxWristVelocityRPM / 0.1; // RPM gained per second
  
  private MotorJointIO wristIO = new MotorJointSparkFlex(wrist, "Wrist", Constants.clawIntakeConstants.wristCANID, 
                                           downSoftRotationLimit, upSoftRotationLimit, true);
  public static AbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder();
  
  private static SparkMaxConfig rollerConfigR = new SparkMaxConfig();
  private static SparkMaxConfig rollerConfigL = new SparkMaxConfig();
  private static SparkFlexConfig wristConfig = new SparkFlexConfig();

  private SparkLimitSwitch forwardLimitSwitch = wrist.getForwardLimitSwitch();
  private SparkLimitSwitch reverseLimitSwitch = wrist.getReverseLimitSwitch();
  public boolean wristForwardStopHit = false;
  public boolean wristBackwardStopHit = false;

  private DoubleSupplier elbowPosSupplier;

  // Cheesy position control for elbow
  private SegmentedMapping upperLimitMapping = new SegmentedMapping();
  private SegmentedMapping lowerLimitMapping = new SegmentedMapping();

  public BooleanSupplier wristForwardStop =
      () -> {
        return forwardLimitSwitch.isPressed();
      };

  public BooleanSupplier wristBackwardStop = 
      () -> {
        return reverseLimitSwitch.isPressed();
      };

  public ClawIntake(DoubleSupplier elbowSupplier) {

    buildLimitMaps();
    coralAcquired = false;
    coralPresentCount = 0;
   elbowPosSupplier = elbowSupplier;
    // Initialize inputs
    this.coralLaserInputs = new LaserCANIOInputsAutoLogged();
    this.wristInOutData = new MotorJointIOInputsAutoLogged();

    rollerConfigR.idleMode(IdleMode.kBrake);
    rollerConfigR.inverted(true); // This makes positive voltage intake, negative out
    rollerRight.configure(rollerConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfigL.idleMode(IdleMode.kBrake);
    // Geometry of the intake means that both motors should turn the same direction
    // Becuase the coral interacts with different sides of the belt there is an implicit inversion, so no
    // software invert needed.
    rollerConfigL.follow(Constants.clawIntakeConstants.rollerCANID, false);
    rollerLeft.configure(rollerConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.smartCurrentLimit(40);
    wristConfig.voltageCompensation(12); 
    wristConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    wristConfig.softLimit.forwardSoftLimitEnabled(false);
                         //.forwardSoftLimit(upSoftRotationLimit);
    wristConfig.softLimit.reverseSoftLimitEnabled(false);
                         //.reverseSoftLimit(downSoftRotationLimit);
    wristConfig.absoluteEncoder.zeroCentered(true);
    wristConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      // Position control 2 and 0.0020 slow.... 2.5 and 0.005 better
      .p(2.5, ClosedLoopSlot.kSlot0)
      .i(0, ClosedLoopSlot.kSlot0)
      .d(0.0050, ClosedLoopSlot.kSlot0)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    wristIO.updateInputs(wristInOutData);
    seabassIO.updateInputs(coralLaserInputs);

    // Combine Hard limit wrist check with soft limit results:
    //wristForwardStopHit = wristForwardStop.getAsBoolean() || wristInOutData.upperSoftLimitHit;
    //wristBackwardStopHit = wristBackwardStop.getAsBoolean() || wristInOutData.lowerSoftLimitHit;
    
   // if (wristForwardStopHit || wristBackwardStopHit) {
    //  wrist.set(0.0); // Stop imediately regardless of the running command
    //}

    // Check for coral
    checkIntakeForCoral();

    LogUtil.logData(wristIO.getName(), wristInOutData);
    LogUtil.logData(seabassIO.getName(), coralLaserInputs);

  }
  
  public void setPositionSetPoint(double pointSet){     //public void setPositionSetPoint(double pointSet){

  double elbowEncoderValue = elbowPosSupplier.getAsDouble() ;  //2.29= gear ratio difference *-.32
  double BoundpointSet = pointSet;
  //double WristBackwardDeflection = -0.0839;
  //double WristSoftStop = elbowEncoderValue  * -0.45 * 0.90; //+ WristBackwardDeflection 
  //if (pointSet < WristSoftStop) {
  //  BoundpointSet = WristSoftStop;
 // }
  double wristRatio = 82.0 /.207; // Convert to angle ratio constant
  double elbowRatio = 90.0 /.423; // Convert to angle ratio
    wristInOutData.elbowAnglePosition = elbowEncoderValue; //* elbowRatio;
    wristInOutData.wristAnglePosition = wristInOutData.absolutePosition * wristRatio;
    wristInOutData.positionSetPoint = BoundpointSet;
    wristInOutData.velocitySetPoint = 0;
    wristInOutData.voltageSetPoint = 0;
   // wristInOutData.wristSoftStop = WristSoftStop;

    double upperSoftStop = (upperLimitMapping.mapPoint(wristInOutData.elbowAnglePosition - 0.05) );/// wristRatio) + 0.0035;
    double lowerSoftStop = lowerLimitMapping.mapPoint(wristInOutData.elbowAnglePosition);// / wristRatio;
 
    wristInOutData.wristSoftStop = upperSoftStop;

    if (pointSet < upperSoftStop) // Wrist back/up is negative
    {
        BoundpointSet = upperSoftStop;
    }
    else if (pointSet >  lowerSoftStop) // Write forward/down is positive
    {
        BoundpointSet = lowerSoftStop;
    }

    wristController.setReference(BoundpointSet, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // Determine coral intake status
  private void checkIntakeForCoral() {
    if (coralLaserInputs.laserConnected &&
         (coralLaserInputs.status == 0)) { // LASERCAN_STATUS_VALID_MEASUREMENT)) {

      if (coralLaserInputs.distance_mm < intakeWidth_mm - GameConstants.coralPieceHalfWidth_mm) {
        // Delay until coral is "fully" seated
        if (coralPresentCount < coralAcquiredCountThreshold) {
          coralPresentCount++;
        }      
      }
      // Otherwise decrement the coral present count (min 0), since the coral was not detected
      else if (coralPresentCount > 0) {
          coralPresentCount--;
      }

      coralAcquired = (coralPresentCount == coralAcquiredCountThreshold);
    }
    else {
      coralAcquired = false;
    }
  }

  public double getPos(){
    return wristEncoder.getPosition();
  }

  public boolean hasCoral() { return coralAcquired; }

  // Return the offset from intake center in mm.
  // Robot left is negative and robot right is positive
  public double coralIntakeOffset() {
    double offset_mm = 0.0;
    if (coralLaserInputs.laserConnected &&
        coralLaserInputs.status == 0) {
      offset_mm = (intakeWidth_mm/2.0) - coralLaserInputs.distance_mm + GameConstants.coralPieceHalfWidth_mm;
    }

    return offset_mm;
  }

  //roller basics
  public void rollerRoll(boolean go) {
    if (go) {rollerRight.setVoltage(7);} //in
    else {rollerRight.setVoltage(0);}
  }

  public void rollerRollBack(boolean roll) {
    if (roll) {rollerRight.setVoltage(-7);} // out
    else {rollerRight.setVoltage(0);}
  }
  
  // Provide joint position in rotations from absolute encoder zero
  public void turnWristToPosition(double startPosition, double targetPosition) {
    double cmdPosition = targetPosition;
   /* if (targetPosition <= upSoftRotationLimit) {
      cmdPosition = upSoftRotationLimit;
    }
    else if (targetPosition >= downSoftRotationLimit) {
      cmdPosition = downSoftRotationLimit;
    }*/
    setPositionSetPoint(targetPosition);
  }

  //wrist basics
  public void wristTurn(boolean forward) {

    double cmdPosition = wristInOutData.absolutePosition;

    if (forward) {
      cmdPosition += 0.125 / 50; // Advance one degree per second (1/50th of a degree per tick)
    }

    //turnWristToPosition(wristInOutData.absolutePosition, cmdPosition);
   //  double range = upSoftRotationLimit - downSoftRotationLimit;
   // turnWristToPosition(wristInOutData.absolutePosition, cmdPosition);

     if (forward) {
      turnWristToPosition(wristInOutData.absolutePosition, upSoftRotationLimit / 2);
    }
    else {
      turnWristToPosition(wristInOutData.absolutePosition, wristInOutData.absolutePosition);
    }
  }

  public void wristTurnBack(boolean backwards) {
    double cmdPosition = wristInOutData.absolutePosition;

    if (backwards) {
      cmdPosition -= 0.125 / 50; // Retreat one degree
    }
    //0.78 to .99
   // turnWristToPosition(wristInOutData.absolutePosition, cmdPosition);
  // double range = upSoftRotationLimit - downSoftRotationLimit;
    turnWristToPosition(wristInOutData.absolutePosition, downSoftRotationLimit /2);
  //  turnWristToPosition(wristInOutData.absolutePosition, cmdPosition);
  }

  public void logPose(String prefix, int snapshotId) {
      Logger.recordOutput(
          prefix + "/Wrist/absolutePosition", wristInOutData.absolutePosition);
  }

  private void buildLimitMaps()
  {
    upperLimitMapping.addPoint( 0.40970999002456665, -0.19605249166488647);
    upperLimitMapping.addPoint( 0.40393054485321045, -0.19605249166488647);
    upperLimitMapping.addPoint( 0.3550521731376648, -0.1756407022476196);
    upperLimitMapping.addPoint( 0.31996268033981323, -0.1628791093826294);
    upperLimitMapping.addPoint( 0.28971415758132935, -0.14921331405639648 );
    upperLimitMapping.addPoint( 0.2653238773345947, -0.13559061288833618);
    upperLimitMapping.addPoint( 0.20975476503372192, -0.10240137577056885);
    upperLimitMapping.addPoint( 0.16785788536071777, -0.07314079999923706);
    upperLimitMapping.addPoint( 0.16292601823806763, -0.07212173938751221);
    upperLimitMapping.addPoint( 0.12783288955688477, -0.04779398441314697);
    upperLimitMapping.addPoint( 0.08687376976013184, -0.021508395671844482);
    upperLimitMapping.addPoint( 0.053716301918029785, -0.0008720159530639648);
    upperLimitMapping.addPoint( 0.019466102123260498, 0.018621087074279785);
    upperLimitMapping.addPoint( 0.0068323612213134766, 0.02343428134918213);
   // upperLimitMapping.addPoint(0.008699741769, 12.377220075487);
   // upperLimitMapping.addPoint(0.00, 12.38);

    lowerLimitMapping.addPoint(0, 2.0);
    lowerLimitMapping.addPoint(2,4);
  }

}
