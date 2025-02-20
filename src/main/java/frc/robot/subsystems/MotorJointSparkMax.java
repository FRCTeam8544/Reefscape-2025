package frc.robot.subsystems;

public class MotorJointSparkMax implements MotorJointIO {

    final private String jointName;
    final private int canId;

    public MotorJointSparkMax(String jointName, int canId) {
      this.jointName = jointName;
      this.canId = canId;
    }

    public String getName() {
      return jointName;
    }

    public void updateInputs(MotorJointIOInputs inputs) {
      inputs.connected = false;
      inputs.absolutePosition = 0.0;
      inputs.externalPosition = 0.0;
      //inputs.lowerLimitHit = false;
      //inputs.upperLimitHit = false;
      inputs.lowerSoftLimitHit = false;
      inputs.upperSoftLimitHit = false;
    }

    public void setPosition(double position) {}
}
