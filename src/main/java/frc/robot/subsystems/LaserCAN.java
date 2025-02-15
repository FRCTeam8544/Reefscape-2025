package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import frc.robot.subsystems.LaserCANIO.LaserCANIOInputs;

public class LaserCAN implements LaserCANIO {
  private final LaserCan laser;
  private final String name;

  public LaserCAN(String name, int canId, FieldOfView fov) {
    this.laser = new LaserCan(canId);
    this.name = name;

    int fovWidth = 0;
    int fovHeight = 0;

    switch (fov) {
      case POINT_2_BY_2:
        fovWidth = 2;
        fovHeight = 2;
        break;
      case NARROW_4_BY_4:
        fovWidth = 4;
        fovHeight = 4;
        break;
      case MEDIUM_8_BY_8:
        fovWidth = 8;
        fovHeight = 8;
        break;
      case WIDE_16_BY_16:
        fovWidth = 16;
        fovHeight = 16;
        break;
    }

    int fovX = fovWidth / 2;
    int fovY = fovHeight / 2;

    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
    // GrappleHook
    try {
      laser.setRangingMode(LaserCan.RangingMode.SHORT);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(fovX, fovY, fovWidth, fovHeight));
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println(
          "LaserCAN(" + String.valueOf(canId) + ") Configuration failed for " + name + "! " + e);
    }
  }

  public void updateInputs(LaserCANIOInputs inputs) {

    LaserCan.Measurement measurement = laser.getMeasurement();

    if (measurement != null) {
      inputs.laserConnected = true;
      inputs.ambient = measurement.ambient;
      inputs.status = measurement.status;
      inputs.distance_mm = measurement.distance_mm;
    } else {
      inputs.laserConnected = false;
      inputs.ambient = 0;
      inputs.status = LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS;
      inputs.distance_mm = 0;
    }

    // TODO Add more logic to reject measurements on errors?
  }

  public String getName() {
    return name;
  }
}
