package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.Faults;

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

      inOutData.connected = true;
      
      inOutData.zeroOffset = 0;
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

      inOutData.velocity = absoluteEncoder.getVelocity();
      inOutData.rawAbsolutePosition = absoluteEncoder.getPosition();
      inOutData.absolutePosition = inOutData.rawAbsolutePosition;
      inOutData.rawExternalPosition = 0;
      inOutData.externalPosition = inOutData.rawExternalPosition;
      inOutData.lowerLimitHit = reverseLimitSwitch.isPressed();
      inOutData.upperLimitHit = forwardLimitSwitch.isPressed();
      inOutData.lowerSoftLimitHit = inOutData.absolutePosition < lowerSoftLimitValue;
      inOutData.upperSoftLimitHit = inOutData.absolutePosition > upperSoftLimitValue;
    }

}
