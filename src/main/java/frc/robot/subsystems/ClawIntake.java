// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.subsystems.LaserCAN;
import frc.robot.subsystems.LaserCANIO;
import frc.robot.subsystems.LaserCANIO.LaserCANIOInputs;
import frc.robot.subsystems.LaserCANIOInputsAutoLogged;

import frc.robot.subsystems.MotorJointSparkFlex;
import frc.robot.subsystems.MotorJointIO.MotorJointIOInputs;
import frc.robot.subsystems.MotorJointIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

public class ClawIntake extends SubsystemBase {
  
  private boolean coralAcquired;
  private static LaserCANIOInputsAutoLogged laserInputs;
  private static LaserCANIO seabass = new LaserCAN("CoralIntake",
                                             Constants.clawIntakeConstants.laser1CANID,
                                             LaserCAN.FieldOfView.NARROW_4_BY_4);

  /** Creates a new ClawIntake. */
  private static SparkMax rollerRight = new SparkMax(Constants.clawIntakeConstants.rollerCANID, MotorType.kBrushed);
  private static SparkMax rollerLeft = new SparkMax(Constants.clawIntakeConstants.roller2CANID, MotorType.kBrushed);
  
  private final double upSoftRotationLimit = Math.toRadians(10);
  private final double downSoftRotationLimit = Math.toRadians(-10);
  private MotorJointIOInputs wristInputs;
  private static SparkFlex wrist = new SparkFlex(Constants.clawIntakeConstants.wristCANID, MotorType.kBrushless);
  private MotorJointIO wristIO = new MotorJointSparkFlex(wrist, "Wrist", Constants.clawIntakeConstants.wristCANID, 
                                           downSoftRotationLimit, upSoftRotationLimit);
  
  private static SparkMaxConfig rollerConfigR = new SparkMaxConfig();
  private static SparkMaxConfig rollerConfigL = new SparkMaxConfig();
  private static SparkFlexConfig wristConfig = new SparkFlexConfig();


  private static DigitalInput limitUpSwitch =
      new DigitalInput(Constants.clawIntakeConstants.wristUpLimitPort);
  private static DigitalInput limitDownSwitch =
      new DigitalInput(Constants.clawIntakeConstants.wristDownLimitPort);
  public boolean wristForwardStopHit = false;
  public boolean wristBackwardStopHit = false;

  public BooleanSupplier wristForwardStop =
      () -> {
        return limitUpSwitch.get();
      };

  public BooleanSupplier wristBackwardStop = 
      () -> {
        return limitDownSwitch.get();
      };

  public ClawIntake() {
    coralAcquired = false;

    // Initialize inputs
    this.laserInputs = new LaserCANIOInputsAutoLogged();
    this.wristInputs = new MotorJointIOInputsAutoLogged();

    rollerConfigR.idleMode(IdleMode.kBrake);
    rollerRight.configure(
        rollerConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfigL.idleMode(IdleMode.kBrake);
    rollerConfigL.follow(Constants.clawIntakeConstants.rollerCANID, true);
    rollerLeft.configure(
        rollerConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.smartCurrentLimit(10);
    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    wristIO.updateInputs(wristInputs);
   
    // Combine Hard limit wrist check with soft limit results:
    // TODO hook limit switches to the spark controllers directly???
    // What about feedback to the software to let it know that a limit has been reached?
    wristForwardStopHit = wristForwardStop.getAsBoolean();// || wristInputs.upperSoftLimitHit;

    wristBackwardStopHit = wristBackwardStop.getAsBoolean();// || wristInputs.lowerSoftLimitHit;

    //if (wristForwardStopHit || wristBackwardStopHit) {
    //  wrist.set(0.0); // Stop imediately regardless of the running command
    //}

    // Log summary data
    Logger.recordOutput(
        "Wrist/connected", wristInputs.connected);
    Logger.recordOutput(
        "Wrist/Measurement/absolutionPosition", wristInputs.absolutePosition);
    Logger.recordOutput(
        "Wrist/Measurement/externalPosition", wristInputs.externalPosition);
    Logger.recordOutput(
        "Wrist/Measurement/lowerSoftLimitHit", wristInputs.lowerSoftLimitHit);
    Logger.recordOutput(
        "Wrist/Measurement/lowerSoftLimitHit", wristInputs.upperSoftLimitHit);
    Logger.recordOutput(
        "Wrist/SetPoint/position", wristInputs.positionSetPoint);

    // Check for coral
    checkCoralIntake();
  }

  // Determine corlal intake status
  private void checkCoralIntake() {
    
    // Handle laser for coral intake
    seabass.updateInputs(laserInputs);

    if (laserInputs.laserConnected && (laserInputs.status == 0)) { // LASERCAN_STATUS_VALID_MEASUREMENT)) {
      coralAcquired = (laserInputs.distance_mm < 240);
    }
    else {
      coralAcquired = false;
    }
    
    // Log summary data
    Logger.recordOutput(
        "Laser/" + seabass.getName() + "/connected", laserInputs.laserConnected);
    Logger.recordOutput("Laser/" + seabass.getName() + "/Measurement/ambient", laserInputs.ambient);
    Logger.recordOutput("Laser/" + seabass.getName() + "/Measurement/status", laserInputs.status);
    Logger.recordOutput(
      "Laser/" + seabass.getName() + "/Measurement/distance_mm", laserInputs.distance_mm);
    
  }

  public void rollerRoll(boolean go) {
    // Roll until the coral is acquired
    if (go && !coralAcquired) {rollerRight.setVoltage(7);} //in
    else {rollerRight.setVoltage(0);}
  }

  public void rollerRollBack(boolean roll) {
    if (roll) {
      rollerRight.setVoltage(7);
    } // out
    else {
      rollerRight.setVoltage(0);
    }
  }

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
