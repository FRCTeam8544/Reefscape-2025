// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// this is like fake for now...
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  public final AddressableLED leds = new AddressableLED(0);
  public final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60); // set length

  public LEDs() {
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
