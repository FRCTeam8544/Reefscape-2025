// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// this is like fake for now...
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  
  Spark spark = new Spark(1);

  public LEDs() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
 // spark.set(0.89);
  }
  public void clawIntake(){
    spark.set(0.77);
  }
  
  public void violet(){
  spark.set(0.11); //0.89
  }

  public void coralAquired(){
  spark.set(0.43);
  }
}
