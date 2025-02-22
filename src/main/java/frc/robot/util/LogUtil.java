package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.MotorJointIO.MotorJointIOInputs;
import frc.robot.subsystems.LaserCANIO.LaserCANIOInputs;

public class LogUtil {
    public static void logData(String prefix, MotorJointIOInputs inOutData) {
      // Log summary data
      Logger.recordOutput(
          prefix + "/connected", inOutData.connected);
      Logger.recordOutput(
          prefix + "/Measurement/absolutePosition", inOutData.absolutePosition);
      Logger.recordOutput(
          prefix + "/Measurement/externalPosition", inOutData.externalPosition);
      Logger.recordOutput(
          prefix + "/Measurement/lowerSoftLimitHit", inOutData.lowerSoftLimitHit);
      Logger.recordOutput(
          prefix + "/Measurement/upperSoftLimitHit", inOutData.upperSoftLimitHit);
      Logger.recordOutput(
          prefix + "/Measurement/lowerLimitHit", inOutData.lowerLimitHit);
      Logger.recordOutput(
          prefix + "/Measurement/upperLimitHit", inOutData.upperLimitHit);
      Logger.recordOutput(
          prefix + "/SetPoint/position", inOutData.positionSetPoint);
      Logger.recordOutput(
          prefix + "/SetPoint/velocity", inOutData.velocitySetPoint);
    }

    public static void logData(String prefix, LaserCANIOInputs inputs)
    {
      Logger.recordOutput(
        prefix + "/connected", inputs.laserConnected);
      Logger.recordOutput(prefix + "/Measurement/ambient", inputs.ambient);
      Logger.recordOutput(prefix + "/Measurement/status", inputs.status);
      Logger.recordOutput(prefix + "/Measurement/distance_mm", inputs.distance_mm);
    }
}
