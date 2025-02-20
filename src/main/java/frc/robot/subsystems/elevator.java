// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.MotorJointSparkFlex;
import frc.robot.subsystems.MotorJointIO.MotorJointIOInputs;
import frc.robot.subsystems.MotorJointIOInputsAutoLogged;

public class elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private static SparkFlex motorController =
      new SparkFlex(Constants.elevatorConstants.rightElevatorCANID, MotorType.kBrushless);

  private static SparkFlex leftMotorController =
      new SparkFlex(Constants.elevatorConstants.leftElevatorCANID, MotorType.kBrushless);
  private static SparkFlex spinMotorRight =
      new SparkFlex(Constants.elevatorConstants.rightElbowCANID, MotorType.kBrushless);
  private static SparkFlex spinMotorLeft =
      new SparkFlex(Constants.elevatorConstants.leftElbowCANID, MotorType.kBrushless);
  private static SparkFlexConfig motorConfig = new SparkFlexConfig();
  private static SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
  private static SparkFlexConfig spinConfig = new SparkFlexConfig();
  private static SparkFlexConfig leftSpinConfig = new SparkFlexConfig();

  
  final static double upSoftStopValue = Math.toRadians(0);
  final static double downSoftStopValue = Math.toRadians(90);

  private MotorJointIOInputs elevatorInputs;
  private static MotorJointIO elevatorMotorIO = new MotorJointSparkFlex(motorController, "Elevator", Constants.elevatorConstants.rightElevatorCANID, 
                                                                    downSoftStopValue, upSoftStopValue);
   
  private static DigitalInput upLimit =
      new DigitalInput(Constants.elevatorConstants.limitSwitchPort); // limit switches
  private static DigitalInput downLimit =
      new DigitalInput(Constants.elevatorConstants.limitSwitch2Port);
  public boolean upStopHit;
  public boolean downStopHit;

  public BooleanSupplier upStop =
      () -> {
        return upLimit.get();
      };
  public BooleanSupplier downStop =
      () -> {
        return downLimit.get();
      };

  public elevator() {
    motorConfig.idleMode(IdleMode.kBrake);
    motorController.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorConfig.inverted(true);

    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.follow(Constants.elevatorConstants.rightElevatorCANID);
    leftMotorController.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    elevatorMotorIO.updateInputs(elevatorInputs);
   
    // Combine Hard limit wrist check with soft limit results:
    // TODO hook limit switches to the spark controllers directly???
    // What about feedback to the software to let it know that a limit has been reached?
    upStopHit = upStop.getAsBoolean() || elevatorInputs.upperSoftLimitHit;

    downStopHit = downStop.getAsBoolean() || elevatorInputs.lowerSoftLimitHit;

  // Hard limits
    // Combine Hard limit wrist check with soft limit results:
    // TODO hook limit switches to the spark controllers directly???
    // What about feedback to the software to let it know that a limit has been reached?
    upStopHit = upStop.getAsBoolean() || elevatorInputs.upperSoftLimitHit;
    downStopHit = downStop.getAsBoolean() || elevatorInputs.lowerSoftLimitHit;

    //if (upStopHit || downStopHit) {
    //  motorController.set(0.0); // Stop imediately regardless of the running command
    //}

    // Log summary data
    Logger.recordOutput(
        "Wrist/connected", elevatorInputs.connected);
    Logger.recordOutput(
        "Wrist/Measurement/absolutionPosition", elevatorInputs.absolutePosition);
    Logger.recordOutput(
        "Wrist/Measurement/externalPosition", elevatorInputs.externalPosition);
    Logger.recordOutput(
        "Wrist/Measurement/lowerSoftLimitHit", elevatorInputs.lowerSoftLimitHit);
    Logger.recordOutput(
        "Wrist/Measurement/lowerSoftLimitHit", elevatorInputs.upperSoftLimitHit);
    Logger.recordOutput(
        "Wrist/SetPoint/position", elevatorInputs.positionSetPoint);

  
  }

  public void spin() {
    spinConfig.idleMode(IdleMode.kBrake);
    spinMotorRight.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spinConfig.inverted(true);

    leftSpinConfig.idleMode(IdleMode.kBrake);
    spinMotorLeft.configure(leftSpinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftSpinConfig.follow(Constants.elevatorConstants.rightElbowCANID);
  }

  public void elevatorMove(boolean up) {
    if (!upStopHit && up) {
      motorController.set(.15);
    }
    if (upStopHit || !up) {
      motorController.set(0);
    }
  }

  public void elevatorLow(boolean down) {
    if (!downStopHit && down) {
      motorController.set(-.15);
    }
    if (downStopHit || !down) {
      motorController.set(0);
    }
  }

  public static void spinElbowForward(boolean go) {
    if (go) {spinMotorRight.set(.15);} 
    else {spinMotorRight.set(0);}
  }

  public static void spinElbowBackwards(boolean execute) {
    if (execute) {spinMotorRight.set(-.15);} 
    else {spinMotorRight.set(0);}
  }
}
