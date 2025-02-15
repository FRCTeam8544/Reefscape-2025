package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Laser extends SubsystemBase {

  private LaserCANIO[] io;
  private LaserCANIOInputsAutoLogged[] inputs;
  private Alert[] disconnectedAlerts;

  public void Laser(LaserCANIO... io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new LaserCANIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new LaserCANIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Laser (" + io[i].getName() + ") index " + Integer.toString(i) + " is disconnected.",
              AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Laser/Sensor" + Integer.toString(i), inputs[i]);

      // Log summary data
      Logger.recordOutput(
          "Laser/" + io[i].getName() + "/Measurement/connected", inputs[i].laserConnected);
      Logger.recordOutput("Laser/" + io[i].getName() + "/Measurement/ambient", inputs[i].ambient);
      Logger.recordOutput("Laser/" + io[i].getName() + "/Measurement/status", inputs[i].status);
      Logger.recordOutput(
          "Laser/" + io[i].getName() + "/Measurement/distance_mm", inputs[i].distance_mm);
    }
  }
}
