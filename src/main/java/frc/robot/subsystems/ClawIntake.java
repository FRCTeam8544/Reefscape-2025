// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
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

public class ClawIntake extends SubsystemBase {
  /** Creates a new ClawIntake. */
  public static SparkMax rollerRight = new SparkMax(Constants.clawIntakeConstants.rollerCANID, MotorType.kBrushed);
  public static SparkMax rollerLeft = new SparkMax(Constants.clawIntakeConstants.roller2CANID, MotorType.kBrushed);
  public static SparkFlex wrist = new SparkFlex(Constants.clawIntakeConstants.wristCANID, MotorType.kBrushless);
  public static LaserCan laser = new LaserCan(Constants.clawIntakeConstants.laser1CANID);
  private static SparkMaxConfig rollerConfigR = new SparkMaxConfig();
  private static SparkMaxConfig rollerConfigL = new SparkMaxConfig();
  private static SparkFlexConfig wristConfig = new SparkFlexConfig();
  private static DigitalInput limit = new DigitalInput(Constants.clawIntakeConstants.wristLimitPort);
  public boolean wristStopHit;

  public BooleanSupplier wristStop =
      () -> {
        return limit.get();
      };

  public ClawIntake() {
    rollerConfigR.idleMode(IdleMode.kBrake);
    rollerRight.configure(rollerConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfigL.idleMode(IdleMode.kBrake);
    rollerConfigL.follow(Constants.clawIntakeConstants.rollerCANID, true);
    rollerLeft.configure(rollerConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristConfig.idleMode(IdleMode.kBrake);
    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (wristStop.getAsBoolean()) {wristStopHit = true;} 
    else {wristStopHit = false;}
  }

  public void rollerRoll(boolean go) {
    if (go) {rollerRight.setVoltage(7);} //in
    else {rollerRight.setVoltage(0);}
  }

  public void rollerRollBack(boolean roll) {
    if (roll) {rollerRight.setVoltage(7);} //out
    else {rollerRight.setVoltage(0);}
  }

  public void wristTurn(boolean forward) {
    if (forward && !wristStopHit) {wrist.set(.1);} 
    else {wrist.set(0);}
  }

  public void wristTurnBack(boolean backwards) {
    if (backwards && !wristStopHit) {wrist.set(-.1);} 
    else {wrist.set(0);}
  }
}
