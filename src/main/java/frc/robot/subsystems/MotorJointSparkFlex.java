package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.Faults;

import frc.robot.Constants;
import frc.robot.subsystems.MotorJointIO;

public class MotorJointSparkFlex implements MotorJointIO {
    
    final private String jointName;
    final private int canId;

    private SparkFlex controller;
    private SparkLimitSwitch forwardLimitSwitch;
    private SparkLimitSwitch reverseLimitSwitch;
    private double lowerSoftLimitValue;
    private double upperSoftLimitValue;
    private boolean useAlternateLimits;
    private double altLowerSoftLimitValue;
    private double altUpperSoftLimitValue;
    private boolean useAbsoluteEncoder;
    private RelativeEncoder externalEncoder;
    private SparkAbsoluteEncoder absoluteEncoder;
    private double zeroOffset;
    
    public MotorJointSparkFlex(SparkFlex controller, String jointName, int canId,
                               double lowerSoftLimitValue, double upperSoftLimitValue,
                               boolean useAbsEncoder) {
      this.jointName = jointName;
      this.canId = canId;
      this.zeroOffset = 0;

      this.controller = controller;
      this.useAbsoluteEncoder = useAbsEncoder;
      this.forwardLimitSwitch = controller.getForwardLimitSwitch();
      this.reverseLimitSwitch = controller.getReverseLimitSwitch();
      if (useAbsoluteEncoder) {
        this.absoluteEncoder = controller.getAbsoluteEncoder();
      }
      else {
        this.externalEncoder = controller.getExternalEncoder();
      }

      this.lowerSoftLimitValue = lowerSoftLimitValue;
      this.upperSoftLimitValue = upperSoftLimitValue;
      this.useAlternateLimits = false;
      this.altLowerSoftLimitValue = lowerSoftLimitValue;
      this.altUpperSoftLimitValue = upperSoftLimitValue;
    }

    public String getName() {
      return jointName;
    }

    public boolean setAlternateLimits(double altLowerLimitValue, double altUpperLimitValue) {
      if ((altLowerLimitValue < altUpperLimitValue) && 
          (altLowerLimitValue >= lowerSoftLimitValue) &&
          (altUpperLimitValue <= upperSoftLimitValue) ) {
        this.altLowerSoftLimitValue = altLowerLimitValue;
        this.altUpperSoftLimitValue = altUpperLimitValue;
        return true;
      }
      return false;
    }

    public void clearAlternateLimits() {
       useAlternateLimits = false;
       this.altLowerSoftLimitValue = lowerSoftLimitValue;
       this.altUpperSoftLimitValue = upperSoftLimitValue;
    }

    public void setZeroOffset(double zeroOffset) {
      this.zeroOffset = zeroOffset;
    }

    public void updateInputs(MotorJointIOInputs inOutData) {
      inOutData.connected = true;
      
      inOutData.zeroOffset = zeroOffset;
      inOutData.motorTemperature = controller.getMotorTemperature();
      inOutData.outputDuty = controller.getAppliedOutput();
      inOutData.busVoltage = controller.getBusVoltage();
      inOutData.outputCurrent = controller.getOutputCurrent();
      inOutData.accumulatedIterm = controller.getClosedLoopController().getIAccum();

      Faults faults = controller.getFaults();
      inOutData.faultCan = faults.can;
      inOutData.faultTemperature = faults.temperature;
      inOutData.faultSensor = faults.sensor;
      inOutData.faultGateDriver = faults.gateDriver;
      inOutData.faultEscEeprom = faults.escEeprom;
      inOutData.faultFirmware = faults.firmware;

      if (useAbsoluteEncoder) {
        inOutData.rawAbsolutePosition = absoluteEncoder.getPosition();
        inOutData.absolutePosition = inOutData.rawAbsolutePosition - zeroOffset;
        inOutData.rawExternalPosition = 0;
        inOutData.externalPosition = 0;
        inOutData.velocity = absoluteEncoder.getVelocity();
        if (useAlternateLimits) {
          inOutData.lowerSoftLimitHit = (inOutData.absolutePosition < altLowerSoftLimitValue);
          inOutData.upperSoftLimitHit = (inOutData.absolutePosition < altUpperSoftLimitValue);
        }
        else {
          inOutData.lowerSoftLimitHit = (inOutData.absolutePosition < lowerSoftLimitValue);
          inOutData.upperSoftLimitHit = (inOutData.absolutePosition < upperSoftLimitValue);
        }
      }
      else {
        inOutData.rawExternalPosition = externalEncoder.getPosition();
        inOutData.externalPosition = inOutData.rawExternalPosition - zeroOffset;
        inOutData.rawAbsolutePosition = 0;
        inOutData.absolutePosition = 0;
        inOutData.velocity = externalEncoder.getVelocity();
        if (useAlternateLimits) {
          inOutData.lowerSoftLimitHit = (inOutData.externalPosition < altLowerSoftLimitValue);
          inOutData.upperSoftLimitHit = (inOutData.externalPosition < altUpperSoftLimitValue);
        }
        else {
          inOutData.lowerSoftLimitHit = (inOutData.externalPosition < lowerSoftLimitValue);
          inOutData.upperSoftLimitHit = (inOutData.externalPosition < upperSoftLimitValue);
        }
      }
      inOutData.lowerLimitHit = reverseLimitSwitch.isPressed();
      inOutData.upperLimitHit = forwardLimitSwitch.isPressed();
    }

}
