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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import frc.robot.subsystems.LaserCAN;
import frc.robot.subsystems.LaserCANIO;
import frc.robot.subsystems.LaserCANIO.LaserCANIOInputs;
import frc.robot.subsystems.LaserCANIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ClawIntake extends SubsystemBase {

  
  private boolean coralAcquired;
  private static LaserCANIOInputsAutoLogged laserInputs;
  private static LaserCANIO seabass = new LaserCAN("CoralIntake",
                                             Constants.clawIntakeConstants.laser1CANID,
                                             LaserCAN.FieldOfView.NARROW_4_BY_4);

  /** Creates a new ClawIntake. */
  private static SparkMax rollerRight = new SparkMax(Constants.clawIntakeConstants.rollerCANID, MotorType.kBrushed);
  private static SparkMax rollerLeft = new SparkMax(Constants.clawIntakeConstants.roller2CANID, MotorType.kBrushed);
  private static SparkFlex wrist = new SparkFlex(Constants.clawIntakeConstants.wristCANID, MotorType.kBrushless);
  private static RelativeEncoder extWristEncoder = wrist.getExternalEncoder(); // Relative?
  private static SparkAbsoluteEncoder absWristEncoder = wrist.getAbsoluteEncoder(); // External?
  private static SparkMaxConfig rollerConfigR = new SparkMaxConfig();
  private static SparkMaxConfig rollerConfigL = new SparkMaxConfig();
  private static SparkFlexConfig wristConfig = new SparkFlexConfig();

  private final double upSoftRotationLimit = Math.toRadians(45);
  private final double downSoftRotationLimit = Math.toRadians(0);

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

    rollerConfigR.idleMode(IdleMode.kBrake);
    rollerRight.configure(
        rollerConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfigL.idleMode(IdleMode.kBrake);
    rollerConfigL.follow(Constants.clawIntakeConstants.rollerCANID, true);
    rollerLeft.configure(
        rollerConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristConfig.idleMode(IdleMode.kBrake);
    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    // Soft wrist check
    boolean forwardSoftLimitHit = false;
    boolean backwardSoftLimitHit = false;
    //final double clawRotation = extWristEncoder.getPosition();
    final double clawRotation = absWristEncoder.getPosition();
    
    if (clawRotation > upSoftRotationLimit) {
      forwardSoftLimitHit = true;
    }
    else if (clawRotation < downSoftRotationLimit) {
      backwardSoftLimitHit = true;
    }

    // Combine Hard limit wrist check with soft limit results:
    // TODO hook limit switches to the spark controllers directly???
    // What about feedback to the software to let it know that a limit has been reached?
    wristForwardStopHit = wristForwardStop.getAsBoolean() || forwardSoftLimitHit;

    wristBackwardStopHit = wristBackwardStop.getAsBoolean() || backwardSoftLimitHit;

    //if (wristForwardStopHit || wristBackwardStopHit) {
    //  wrist.set(0.0); // Stop imediately regardless of the running command
    //}

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
