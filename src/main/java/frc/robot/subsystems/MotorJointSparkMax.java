package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class MotorJointSparkMax implements MotorJointIO {

    final private String jointName;
    final private int canId;
    final private double lowerSoftLimitValue;
    final private double upperSoftLimitValue;
    final private SparkMax controller;
    final private RelativeEncoder externalEncoder;
    final private SparkAbsoluteEncoder absoluteEncoder;
    
    public MotorJointSparkMax(SparkMax controller, String jointName, int canId,
                              double lowerSoftLimitValue, double upperSoftLimitValue) {
      this.jointName = jointName;
      this.canId = canId;
      this.upperSoftLimitValue = upperSoftLimitValue;
      this.lowerSoftLimitValue = lowerSoftLimitValue;
      this.controller = controller;
      this.absoluteEncoder = controller.getAbsoluteEncoder();
      this.externalEncoder = controller.getAlternateEncoder();
    }

    public String getName() {
      return jointName;
    }

    public void updateInputs(MotorJointIOInputs inputs) {

      inputs.connected = false;
      inputs.absolutePosition = absoluteEncoder.getPosition();
      inputs.externalPosition = externalEncoder.getPosition();
      //inputs.lowerLimitHit = false;
      //inputs.upperLimitHit = false;
      inputs.lowerSoftLimitHit = inputs.absolutePosition < lowerSoftLimitValue;
      inputs.upperSoftLimitHit = inputs.absolutePosition > upperSoftLimitValue;
      inputs.positionSetPoint = 0.0; // TODO do we need this???
    }

    public void setVelocity(double speed) {
      controller.set(speed);
    }
}
