package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Cameras extends CamerasIO {

  PhotonCamera frontLeftChassis = new PhotonCamera("front-left-chassis-2");
  //PhotonCamera frontRightChassis = new PhotonCamera("front-right-chassis-2");
  //PhotonCamera driverCam = new PhotonCamera("driver-cam");
  PhotonPipelineResult frontLeftChassisResult;
  PhotonPipelineResult frontRightChassisResult;

  public void periodic() {
    // get the latest frame from the cameras
    frontLeftChassisResult = frontLeftChassis.getLatestResult();
    //frontRightChassisResult = frontRightChassis.getLatestResult();
  }
}
