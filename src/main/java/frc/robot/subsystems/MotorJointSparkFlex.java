package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
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
    }

    public String getName() {
      return jointName;
    }

    public void setZeroOffset(double zeroOffset) {
      this.zeroOffset = zeroOffset;
    }

    public void updateInputs(MotorJointIOInputs inOutData) {
      inOutData.connected = true;
      if (useAbsoluteEncoder) {
        inOutData.absolutePosition = absoluteEncoder.getPosition() + zeroOffset;
        inOutData.lowerSoftLimitHit = (inOutData.absolutePosition < lowerSoftLimitValue);
        inOutData.upperSoftLimitHit = (inOutData.absolutePosition < upperSoftLimitValue);
        inOutData.externalPosition = 0; // TODO Can we read this anyways on the flex?
      }
      else {
        inOutData.externalPosition = externalEncoder.getPosition() + zeroOffset;
        inOutData.lowerSoftLimitHit = (inOutData.externalPosition < lowerSoftLimitValue);
        inOutData.upperSoftLimitHit = (inOutData.externalPosition < upperSoftLimitValue);
        inOutData.absolutePosition = 0; // TODO Can we read this anyways on the flex?
      }
      inOutData.lowerLimitHit = reverseLimitSwitch.isPressed();
      inOutData.upperLimitHit = forwardLimitSwitch.isPressed();
    }

    public void setVelocity(double speed) {
        controller.set(speed);
    }
}
