// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /*THESE CLASSES ARE WHERE YOU SET PIDS
  CAN ID guide!! 
  rio is 0
  drive 1-9
  elevator 10-19
  intake 20-29
  climber 31-39
  misc 60-63
  */

  public static final class elevatorConstants{
    public static final int rightElbowCANID = 10;
    public static final int leftElbowCANID = 11;
    public static final int rightElevatorCANID = 12;
    public static final int leftElevatorCANID = 13;
    public static final int limitSwitchPort = 0;
    public static final int limitSwitch2Port = 1;
  }
  public static final class clawIntakeConstants{ 
    public static final int rollerCANID = 20;
    public static final int roller2CANID = 21;
    public static final int wristCANID = 22;
    public static final int laserCANID = 23;
    public static final int laser2CANID = 24;
    public static final int wristLimitPort = 3;
    public static final int laserCANPort = 4;
  }
  public static final class climberConstants{
    public static final int climberCANID = 31;
    public static final int climber2CANID = 32;
  }

}
