package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface MotorJointIO {
    
  @AutoLog
  public static class MotorJointIOInputs {
    // Inputs
    public boolean connected = false;

    public double zeroOffset = 0;
    public double rawExternalPosition = 0;
    public double rawAbsolutePosition = 0;
    public double externalPosition = 0;
    public double absolutePosition = 0;

    public boolean upperSoftLimitHit = false;
    public boolean lowerSoftLimitHit = false;
    public boolean upperLimitHit = false;
    public boolean lowerLimitHit = false;

    // Outputs
    public double positionSetPoint = 0.0; // Rotations in radians
    public double velocitySetPoint = 0.0; // Percent of max motor speed (0...1)
  }

  public String getName();

  public default boolean setAlternateLimits(double altLowerLimitValue, double altUpperLimitValue) {
    return false;
  };

  public default void clearAlternateLimits() {};

  public default void setZeroOffset(double zeroOffset) {};

  public void updateInputs(MotorJointIOInputs inOutData);

  // Set speed in percent of max motor speed
  public void setVelocity(double speed);

}
