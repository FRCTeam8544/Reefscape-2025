package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.TimedRobot;

public class LaserCAN extends TimedRobot {
  private LaserCan laser;

  @Override
  public void robotInit() {
    laser = new LaserCan(9);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in
    // GrappleHook
    try {
      laser.setRangingMode(LaserCan.RangingMode.SHORT);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void robotPeriodic() {
    LaserCan.Measurement measurement = laser.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println(
          "Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an
      // unreliable measurement.
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
