package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface MotorJointIO {
    
  @AutoLog
  public static class MotorJointIOInputs {
    // Inputs
    public boolean connected = false;
    public double wristAnglePosition = 0;
    public double elbowAnglePosition = 0;
    public double zeroOffset = 0;
    public double rawExternalPosition = 0;
    public double rawAbsolutePosition = 0;
    public double externalPosition = 0;
    public double absolutePosition = 0;
    public double velocity = 0;

    public double motorTemperature = 0;

    // Fault codes
    public boolean faultSensor;
    public boolean faultCan;
    public boolean faultTemperature;
    public boolean faultGateDriver;
    public boolean faultEscEeprom;
    public boolean faultFirmware;
    
    // Limits
    public boolean upperSoftLimitHit = false;
    public boolean lowerSoftLimitHit = false;
    public boolean upperLimitHit = false;
    public boolean lowerLimitHit = false;

    // Outputs
    public double busVoltage = 0;
    public double accumulatedIterm = 0;
    public double outputDuty = 0; // -1 to 1 percent applied of bus voltage
    public double outputCurrent = 0;

    public double positionSetPoint = 0.0; // Rotations in radians
    public double velocitySetPoint = 0.0; // Percent of max motor speed (0...1)
    public double voltageSetPoint = 0.0; // Motor voltage, usually not directly controlled
    
  }

  public String getName();

  public default boolean setAlternateLimits(double altLowerLimitValue, double altUpperLimitValue) {
    return false;
  };

  public default void clearAlternateLimits() {};

  public default void setZeroOffset(double zeroOffset) {};

  public void updateInputs(MotorJointIOInputs inOutData);

}
