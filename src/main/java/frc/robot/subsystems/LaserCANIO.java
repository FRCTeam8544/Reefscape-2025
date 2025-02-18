package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface LaserCANIO {

  public enum FieldOfView {
    POINT_2_BY_2,
    NARROW_4_BY_4,
    MEDIUM_8_BY_8,
    WIDE_16_BY_16
  }

  @AutoLog
  public static class LaserCANIOInputs {
    public boolean laserConnected = false;
    public int distance_mm = 0;
    public int ambient = 0;
    public int status = 0;
  }

  public default void updateInputs(LaserCANIOInputs inputs) {}

  public String getName();
}
