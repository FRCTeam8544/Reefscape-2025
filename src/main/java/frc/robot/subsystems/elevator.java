// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
<<<<<<< Updated upstream
<<<<<<< Updated upstream
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.SparkClosedLoopController; //pid 
=======
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
>>>>>>> Stashed changes
=======
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

public class elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private static SparkMax motorController = new SparkMax(10, MotorType.kBrushless);
  private static SparkMax leftMotorController = new SparkMax(11, MotorType.kBrushless);
<<<<<<< Updated upstream
<<<<<<< Updated upstream
  //private RelativeEncoder elevatorEncoder = motorController.getAlternateEncoder();
  private static SparkMaxConfig config = new SparkMaxConfig();
  //private SparkClosedLoopController motorPid = motorController.getClosedLoopController(); //looks like use if no limit switch...
  //private SparkClosedLoopController leftMotorPid = leftMotorController.getClosedLoopController();
=======
  private static SparkMaxConfig config = new SparkMaxConfig();
>>>>>>> Stashed changes
=======
  private static SparkMaxConfig config = new SparkMaxConfig();
>>>>>>> Stashed changes
  private static DigitalInput upLimit = new DigitalInput(0); //limit switches
  private static DigitalInput downLimit = new DigitalInput(1);
  public boolean upStopHit;
  public boolean downStopHit;

  public BooleanSupplier upStop = () -> {
    return upLimit.get();
  }; 
  public BooleanSupplier downStop = () -> {
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
    if(upStop.getAsBoolean()){
      upStopHit = true;}
      else{upStopHit = false;}

    if(downStop.getAsBoolean()){
      downStopHit = true;}
    else{downStopHit = false;}
  }
  
    public void elevatorMove(boolean up){
      if(!upStopHit && up){
        motorController.set(.5);
        leftMotorController.set(.5);}

      if(!downStopHit && !up){
        motorController.set(-.5);
        leftMotorController.set(-.5);

      if(upStopHit || downStopHit){
        motorController.set(0);
        leftMotorController.set(0);
      }
      }
  }
}
