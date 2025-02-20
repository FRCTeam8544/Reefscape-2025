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
  private static RelativeEncoder externalEncoder = motorController.getExternalEncoder(); // Relative?
  private static SparkAbsoluteEncoder absoluteEncoder = motorController.getAbsoluteEncoder(); // External?
 
  
  private static DigitalInput upLimit =
      new DigitalInput(Constants.elevatorConstants.limitSwitchPort); // limit switches
  private static DigitalInput downLimit =
      new DigitalInput(Constants.elevatorConstants.limitSwitch2Port);
  public boolean upStopHit;
  public boolean downStopHit;

  final double upSoftStopValue = Math.toRadians(0);
  final double downSoftStopValue = Math.toRadians(300);

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

    // Soft limits
    boolean upSoftLimitHit = false;
    boolean downSoftLimitHit = false;
    //final double externalElevatorRotations = externalEncoder.getPosition();
    final double absElevatorRotations = absoluteEncoder.getPosition();
    if (absElevatorRotations > upSoftStopValue) {
      upSoftLimitHit = true;
    }
    else if (absElevatorRotations < downSoftStopValue) {
      downSoftLimitHit = true;
    }

    // Hard limits
    // Combine Hard limit wrist check with soft limit results:
    // TODO hook limit switches to the spark controllers directly???
    // What about feedback to the software to let it know that a limit has been reached?
    upStopHit = upStop.getAsBoolean() || upSoftLimitHit;
    downStopHit = downStop.getAsBoolean() || downSoftLimitHit;

    //if (upStopHit || downStopHit) {
    //  motorController.set(0.0); // Stop imediately regardless of the running command
    //}
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
