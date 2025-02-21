package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.subsystems.MotorJointIO;

public class MotorJointSparkFlex implements MotorJointIO {
    
    final private String jointName;
    final private int canId;

    private SparkFlex controller;
    private RelativeEncoder extWristEncoder;
    private SparkAbsoluteEncoder absWristEncoder;
    private double lowerSoftLimitValue;
    private double upperSoftLimitValue;
    
    public MotorJointSparkFlex(SparkFlex controller, String jointName, int canId, 
                               double lowerSoftLimitValue, double upperSoftLimitValue) {
      this.jointName = jointName;
      this.canId = canId;

     // this.controller = new SparkFlex(canId, MotorType.kBrushless);
      this.controller = controller;
      this.extWristEncoder = controller.getExternalEncoder();
      this.absWristEncoder = controller.getAbsoluteEncoder();

      this.lowerSoftLimitValue = lowerSoftLimitValue;
      this.upperSoftLimitValue = upperSoftLimitValue;
    }

    public String getName() {
      return jointName;
    }

    public void updateInputs(MotorJointIOInputs inOutData) {
      inOutData.connected = true;
      inOutData.absolutePosition = absWristEncoder.getPosition();
      inOutData.externalPosition = extWristEncoder.getPosition();
     // inOutData.lowerLimitHit = false;
     // inOutData.upperLimitHit = false;
      inOutData.lowerSoftLimitHit = (inOutData.absolutePosition < lowerSoftLimitValue);
      inOutData.upperSoftLimitHit = (inOutData.absolutePosition < upperSoftLimitValue);
    }

    public void setVelocity(double speed) {
        controller.set(speed);
    }
}
