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

      // SparkMax can only support either an absolute encoder or a external encoder, never both.
      // Pick one encoder object type to instaniate. If you create both the robot code will crash.
      this.absoluteEncoder = controller.getAbsoluteEncoder();
      //this.externalEncoder = controller.getAlternateEncoder();
    }

    public String getName() {
      return jointName;
    }

    public void updateInputs(MotorJointIOInputs inOutData) {

      
      inOutData.connected = false;
      
      inOutData.zeroOffset = 0;
      inOutData.rawAbsolutePosition = absoluteEncoder.getPosition();
      inOutData.absolutePosition = inOutData.rawAbsolutePosition;
      inOutData.rawExternalPosition = 0;
      inOutData.externalPosition = inOutData.rawExternalPosition;
      inOutData.lowerLimitHit = reverseLimitSwitch.isPressed();
      inOutData.upperLimitHit = forwardLimitSwitch.isPressed();
      inOutData.lowerSoftLimitHit = inOutData.absolutePosition < lowerSoftLimitValue;
      inOutData.upperSoftLimitHit = inOutData.absolutePosition > upperSoftLimitValue;

    }

    public void setVelocity(double speed) {
      controller.set(speed);
    }
}
