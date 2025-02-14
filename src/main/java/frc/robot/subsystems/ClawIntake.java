// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClawIntake extends SubsystemBase {
  /** Creates a new ClawIntake. */
  public static SparkMax rollerRight = new SparkMax(Constants.clawIntakeConstants.rollerCANID, MotorType.kBrushless);
  public static SparkMax rollerLeft = new SparkMax(Constants.clawIntakeConstants.roller2CANID, MotorType.kBrushless);
  public static SparkFlex wrist = new SparkFlex(Constants.clawIntakeConstants.wristCANID, MotorType.kBrushless);
  public static LaserCan laser = new LaserCan(Constants.clawIntakeConstants.laserCANID);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static DigitalInput limit = new DigitalInput(Constants.clawIntakeConstants.wristLimitPort);
  public boolean wristStopHit;

  public BooleanSupplier wristStop =
      () -> {
        return limit.get();
      };

  public ClawIntake() {
    rollerRight.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);
    
    rollerLeft.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);
    config.follow(Constants.clawIntakeConstants.rollerCANID, true);

    wrist.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);
  }

  public void rollerRoll(boolean go) {
  if(go){rollerRight.set(.5);}
  else{rollerRight.set(0);}
  }

  public void rollerRollBack(boolean roll){
    if(roll){rollerRight.set(-.5);}
    else{rollerRight.set(0);}
  }

  public void wristTurn(boolean forward){ 
    if(forward && !wristStopHit)wrist.set(.1);
  else{wrist.set(0);}}

  public void wristTurnBack(boolean backwards){
    if(backwards && !wristStopHit){wrist.set(-.1);}
    else{wrist.set(0);}
  }

  @Override
  public void periodic() {
    if (wristStop.getAsBoolean()) {wristStopHit = true;} 
    else {wristStopHit = false;}
    // This method will be called once per scheduler run
  }
}
