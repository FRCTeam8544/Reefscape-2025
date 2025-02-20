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
    private double lowerSoftLimitValue = 0;
    private double upperSoftLimitValue = 360;
    
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

    public void updateInputs(MotorJointIOInputs inputs) {
      inputs.connected = true;
      inputs.absolutePosition = absWristEncoder.getPosition();
      inputs.externalPosition = extWristEncoder.getPosition();
     // inputs.lowerLimitHit = false;
     // inputs.upperLimitHit = false;
      inputs.lowerSoftLimitHit = (inputs.absolutePosition < lowerSoftLimitValue);
      inputs.upperSoftLimitHit = (inputs.externalPosition < upperSoftLimitValue);
    }

    public void setPosition(double position) {
        controller.set(position);
    }
}
