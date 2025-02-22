package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class MotorJointSparkMax implements MotorJointIO {

    final private String jointName;
    final private int canId;
    final private double lowerSoftLimitValue;
    final private double upperSoftLimitValue;
    final private SparkMax controller;
    final private SparkLimitSwitch forwardLimitSwitch;
    final private SparkLimitSwitch reverseLimitSwitch;
    //final private RelativeEncoder externalEncoder;
    final private SparkAbsoluteEncoder absoluteEncoder;
    
    public MotorJointSparkMax(SparkMax controller, String jointName, int canId,
                              double lowerSoftLimitValue, double upperSoftLimitValue) {
      this.jointName = jointName;
      this.canId = canId;
      this.upperSoftLimitValue = upperSoftLimitValue;
      this.lowerSoftLimitValue = lowerSoftLimitValue;
      this.controller = controller;
      this.forwardLimitSwitch = controller.getForwardLimitSwitch();
      this.reverseLimitSwitch = controller.getReverseLimitSwitch();
      this.absoluteEncoder = controller.getAbsoluteEncoder();
      //this.externalEncoder = controller.getAlternateEncoder();
    }

    public String getName() {
      return jointName;
    }

    public void updateInputs(MotorJointIOInputs inOutData) {

      inOutData.connected = false;
      inOutData.absolutePosition = absoluteEncoder.getPosition();
      //inOutData.externalPosition = externalEncoder.getPosition();
      inOutData.lowerLimitHit = reverseLimitSwitch.isPressed();
      inOutData.upperLimitHit = forwardLimitSwitch.isPressed();
      inOutData.lowerSoftLimitHit = inOutData.absolutePosition < lowerSoftLimitValue;
      inOutData.upperSoftLimitHit = inOutData.absolutePosition > upperSoftLimitValue;
    }

    public void setVelocity(double speed) {
      controller.set(speed);
    }
}
