package frc.robot.subsystems;

public class LaserCANSim implements LaserCANIO {
  private String name;

  public LaserCANSim(String name, int canId) {}

  public void updateInputs(LaserCANIOInputs inputs) {
    inputs.laserConnected = true;
    inputs.ambient = 0;
    inputs.status = 2;
    inputs.distance_mm = 0;
  }

  public String getName() {
    return name + ": LaserCANSim not implemented";
  }

}
