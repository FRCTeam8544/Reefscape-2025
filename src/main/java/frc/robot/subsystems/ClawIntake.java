// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import au.grapplerobotics.LaserCan;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

public class ClawIntake extends SubsystemBase {
  
  private final double intakeWidth_mm = 250; // TODO Use the laser to determine this number as built
  private final int coralAcquiredCountThreshold = 3; // Wait 3 * 20 ms before declaring coral acquired

  private boolean coralAcquired;
  private int coralPresentCount;

  private static LaserCANIOInputsAutoLogged coralLaserInputs;
  private static LaserCANIO seabassIO = new LaserCAN("CoralLaser",
                                             Constants.clawIntakeConstants.laser1CANID,
                                             LaserCAN.FieldOfView.NARROW_4_BY_4);

  /*private boolean algaeAcquired;
  private static LaserCANIOInputsAutoLogged algaeLaserInputs;
  private static LaserCANIO sharkIO = new LaserCAN("AlgaeLaser", 
                                                 Constants.clawIntakeConstants.laser2CANID,
                                                 LaserCAN.FieldOfView.WIDE_16_BY_16);*/

  /** Creates a new ClawIntake. */
  private static SparkMax rollerRight = new SparkMax(Constants.clawIntakeConstants.rollerCANID, MotorType.kBrushed);
  private static SparkMax rollerLeft = new SparkMax(Constants.clawIntakeConstants.roller2CANID, MotorType.kBrushed);
  
  private final double upSoftRotationLimit = 0.56; // Rotations
  private final double downSoftRotationLimit = 0;  // Rotations
  private MotorJointIOInputs wristInOutData;
  private static SparkFlex wrist = new SparkFlex(Constants.clawIntakeConstants.wristCANID, MotorType.kBrushless);
  private MotorJointIO wristIO = new MotorJointSparkFlex(wrist, "Wrist", Constants.clawIntakeConstants.wristCANID, 
                                           downSoftRotationLimit, upSoftRotationLimit, true);
  private static AbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder();
  
  private static SparkMaxConfig rollerConfigR = new SparkMaxConfig();
  private static SparkMaxConfig rollerConfigL = new SparkMaxConfig();
  private static SparkFlexConfig wristConfig = new SparkFlexConfig();

  private SparkLimitSwitch forwardLimitSwitch = wrist.getForwardLimitSwitch();
  private SparkLimitSwitch reverseLimitSwitch = wrist.getReverseLimitSwitch();
  public boolean wristForwardStopHit = false;
  public boolean wristBackwardStopHit = false;

  public BooleanSupplier wristForwardStop =
      () -> {
        return forwardLimitSwitch.isPressed();
      };

  public BooleanSupplier wristBackwardStop = 
      () -> {
        return reverseLimitSwitch.isPressed();
      };

  public ClawIntake() {
    coralAcquired = false;
    coralPresentCount = 0;

    // Initialize inputs
    this.coralLaserInputs = new LaserCANIOInputsAutoLogged();
    //this.algaeLaserInputs = new LaserCANIOInputsAutoLogged();
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
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);
    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    wristIO.updateInputs(wristInOutData);
    seabassIO.updateInputs(coralLaserInputs);
    //sharkIO.updateInputs(algaeLaserInputs);

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
   // LogUtil.logData(sharkIO.getName(), algaeLaserInputs);

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

  //auto wrist/rollers
  //again encoders to be set... for all...
  public void wristAuto(boolean a){
    if(a && wristEncoder.getPosition() < .3) {wrist.set(.3);}
    else{wrist.set(0);}
  }

  public void wristAutoRetract() {
    if(wristEncoder.getPosition() > 0.1) {wrist.set(-.3);} 
    else{wrist.set(0);}
  }

  public void intakePose(){
    if(wristEncoder.getPosition() > 0) {wrist.set(-.3);}
    else {wrist.set(0);}
  }

  public void rollersAuto(boolean b) {
    if(b) {rollerRight.setVoltage(7);}
  }

  public void rollersAutoIntake(boolean c) {
    if(c) {rollerRight.setVoltage(-7);}
    else {rollerRight.setVoltage(0);}
  }

  //wrist basics
  public void wristTurn(boolean forward) {
    if (forward && !wristForwardStopHit) {
      wrist.set(.15);
    } else {
      wrist.set(0);
    }
  }

  public void wristTurnBack(boolean backwards) {
    if (backwards && !wristBackwardStopHit) {
      wrist.set(-.15);
    } else {
      wrist.set(0);
    }
  }
}
