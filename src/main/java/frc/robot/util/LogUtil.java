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
          prefix + "/zeroOffset", inOutData.zeroOffset);
      Logger.recordOutput(
          prefix + "/Measurement/rawAbsolutePosition", inOutData.rawAbsolutePosition);
      Logger.recordOutput(
          prefix + "/Measurement/absolutePosition", inOutData.absolutePosition);
      Logger.recordOutput(
          prefix + "/Measurement/wristAnglePosition", inOutData.wristAnglePosition);
      Logger.recordOutput(
          prefix + "/Measurement/elbowAnglePosition", inOutData.elbowAnglePosition);
      Logger.recordOutput(
          prefix + "/Measurement/rawExternalPosition", inOutData.rawExternalPosition);
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
          prefix + "/Measurement/velocity", inOutData.velocity);

      Logger.recordOutput(
            prefix + "/Measurement/busVoltage", inOutData.busVoltage);
      Logger.recordOutput(
            prefix + "/Measurement/temperature", inOutData.motorTemperature);

      Logger.recordOutput(
            prefix + "/SetPoint/appliedOutputDuty", inOutData.outputDuty);
      Logger.recordOutput(
            prefix + "/SetPoint/accumulatedIterm", inOutData.accumulatedIterm);
      Logger.recordOutput(
            prefix + "/SetPoint/outputCurrent", inOutData.outputCurrent);
      
      Logger.recordOutput(
          prefix + "/SetPoint/position", inOutData.positionSetPoint);
      Logger.recordOutput(
          prefix + "/SetPoint/velocity", inOutData.velocitySetPoint);
      Logger.recordOutput(
          prefix + "/SetPoint/voltage", inOutData.voltageSetPoint);
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
