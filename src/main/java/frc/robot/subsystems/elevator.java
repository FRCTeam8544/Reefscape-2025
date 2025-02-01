// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private static SparkMax motorController = new SparkMax(10, MotorType.kBrushless);

  private static SparkMax leftMotorController = new SparkMax(11, MotorType.kBrushless);
  private static SparkMaxConfig config = new SparkMaxConfig();
  private static DigitalInput upLimit = new DigitalInput(0); // limit switches
  private static DigitalInput downLimit = new DigitalInput(1);
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
    leftMotorController.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);

    motorController.configure(config, null, null);
    config.idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (upStop.getAsBoolean()) {
      upStopHit = true;
    } else {
      upStopHit = false;
    }

    if (downStop.getAsBoolean()) {
      downStopHit = true;
    } else {
      downStopHit = false;
    }
  }

  public void elevatorMove(boolean up) {
    if (!upStopHit && up) {
      motorController.set(.2);
      leftMotorController.set(.2);
    }

    if (!downStopHit && !up) {
      motorController.set(-.2);
      leftMotorController.set(-.2);

      if (upStopHit || downStopHit) {
        motorController.set(0);
        leftMotorController.set(0);
      }
    }
  }
}
