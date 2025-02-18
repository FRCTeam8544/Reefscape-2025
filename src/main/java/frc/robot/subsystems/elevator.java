// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.spark.SparkClosedLoopController;
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
  // private static SparkClosedLoopController maxPid = motorController.getClosedLoopController();
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

    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.follow(Constants.elevatorConstants.rightElevatorCANID, true);
    leftMotorController.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spin() {
    spinConfig.idleMode(IdleMode.kBrake);
    spinMotorRight.configure(
        spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftSpinConfig.idleMode(IdleMode.kBrake);
    leftSpinConfig.follow(Constants.elevatorConstants.rightElbowCANID, true);
    spinMotorLeft.configure(
        leftSpinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    upStopHit = upStop.getAsBoolean();

    downStopHit = downStop.getAsBoolean();

    // need last cmd speed to do something here....
    // if (upStopHit || downStopHit) {
    //  runElevatorVelocity(0.0);
    // }

    // TODO add smoothing to reduce velocity when it approaches end stops?
    // Just taper the max velocity at the ends?
    // Same for the runVelocity?
    // Build api to go to specific set points?
    // Use fabrik to target the arm? maybe for auto
    // Or let pid controller handle thjis??? using a positional interface instead of velocityt?

  }

  // Positive is up, negative is down, normalized between 0 to 1
  // Zero is no movement, 1 is max speed for the motor
  public void runElevatorVelocity(double speed) {
    final double maxRunVelocity = 0.4; // TODO what units????

    if (speed >= 0.0) { // Command up velocity requested
      if (!upStopHit) {
        motorController.set(Math.min(speed, maxRunVelocity));
      } else {
        motorController.set(0.0); // Stop motor
      }
    } else { // Commanded speed is negative
      if (!downStopHit) {
        motorController.set(Math.max(speed, -maxRunVelocity));
      } else {
        motorController.set(0.0);
      }
    }
  }

  public void runWristVelocity(double speed) {
    spinMotorRight.set(0.0);
  }

  public void elevatorMove(boolean up) {
    if (!upStopHit && up) {
      motorController.set(.1);
    }

    if (upStopHit || !up) {
      motorController.set(0);
    }
  }

  public void elevatorLow(boolean down) {
    if (!downStopHit && down) {
      motorController.set(-.1);
    }

    if (downStopHit || !down) {
      motorController.set(0);
    }
  }

  public static void spinElbowForward(boolean go) {
    if (go) {
      spinMotorRight.set(.2);
    } else {
      spinMotorRight.set(0);
    }
  }

  public static void spinElbowBackwards(boolean execute) {
    if (execute) {
      spinMotorRight.set(-.2);
    } else {
      spinMotorRight.set(0);
    }
  }
}
