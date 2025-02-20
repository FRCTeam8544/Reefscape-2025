package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface MotorJointIO {
    
  @AutoLog
  public static class MotorJointIOInputs {
    public boolean connected = false;
    public double externalPosition = 0;
    public double absolutePosition = 0;
    public boolean upperSoftLimitHit = false;
    public boolean lowerSoftLimitHit = false;
    public double positionSetPoint = 0.0;
  //  public boolean upperLimitHit = false;
  //  public boolean lowerLimitHit = false;
  }

  public String getName();

  public void updateInputs(MotorJointIOInputs inputs);

  public void setPosition(double position);

}
