// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.WristForward;
import frc.robot.commands.AutonomousAuto.AutoScore;
import frc.robot.commands.AutonomousAuto.FAKEIntakeAuto;
import frc.robot.commands.AutosReal.LILYACHTY;
import frc.robot.commands.AutosReal.THEAUTO;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbBack;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElbowBack;
import frc.robot.commands.RollersForward;
import frc.robot.commands.RollersBack;
import frc.robot.commands.ElbowForward;
import frc.robot.commands.WristBack;
import frc.robot.commands.WristCommand;
import frc.robot.commands.Elevator.elevatorDown;
import frc.robot.commands.Elevator.elevatorUp;
import frc.robot.commands.Elevator.ElevatorAuto2;
import frc.robot.commands.Elevator.ElevatorAuto3;
import frc.robot.commands.Elevator.ElevatorAuto4;
import frc.robot.commands.Elevator.ElevatorCommands;
import frc.robot.commands.Elevator.ElevatorStow;
import frc.robot.commands.Elevator.IntakeAuto;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator = new Elevator();
  private final ClawIntake clawIntake = new ClawIntake();
  private final Climber climber = new Climber();

  // Controller
  private final CommandXboxController romeo = new CommandXboxController(0); // driver
  private final CommandXboxController juliet = new CommandXboxController(1); // smooth operator
  private final Trigger aButton = new Trigger(juliet.a());
  private final Trigger bButton = new Trigger(juliet.b());
  private final Trigger yButton = new Trigger(juliet.y());
  private final Trigger xButton = new Trigger(juliet.x());
  private final Trigger rightBack = new Trigger(juliet.rightBumper());
  private final Trigger leftBack = new Trigger(juliet.leftBumper());
  private final Trigger rightBackTop = new Trigger(juliet.rightTrigger());
  private final Trigger leftBackTop = new Trigger(juliet.leftTrigger());
  private final Trigger startButton = new Trigger(juliet.start());
  private final Trigger backButton = new Trigger(juliet.back()); 
  private final Trigger UpDPad = new Trigger(juliet.pov(0));
  private final Trigger DownDPad = new Trigger(juliet.pov(180));
  

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    
    //named commands for pathplanner
    NamedCommands.registerCommand("AutoScore4", new AutoScore(elevator, clawIntake));
    NamedCommands.registerCommand("IntakePose", new FAKEIntakeAuto(elevator, clawIntake));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        /*vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(leftChassisApriltag, robotToCamera0),
                new VisionIOPhotonVision(rightChassisApriltag, robotToCamera1));*/
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(leftChassisApriltag, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(rightChassisApriltag, robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Take the wok to Poland", new THEAUTO(elevator, clawIntake, drive));
    autoChooser.addOption("Lil yachty", new LILYACHTY(elevator, clawIntake, drive));

    // Set up SysId routines do we need this i think it is messing with my autos...
   /* autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> romeo.getLeftY(), () -> romeo.getLeftX(), () -> -romeo.getRightX()));

    // Lock to 0° when A button is held
    romeo
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -romeo.getLeftY(), () -> -romeo.getLeftX(), () -> new Rotation2d()));

    // Drive constant velocity field relative while held: DPad UP is away from alliance station
    // Specify the angle of the controller D-Pad (POV) 0, 45, 90, 135, 180 ...
    romeo.povUp().whileTrue(DriveCommands.dpadDriveField(drive, 0));
    // romeo.povUpRight().whileTrue(DriveCommands.dpadDriveField(drive, 45));
    romeo.povRight().whileTrue(DriveCommands.dpadDriveField(drive, 90));
    // romeo.povDownRight().whileTrue(DriveCommands.dpadDriveField(drive, 135));
    romeo.povDown().whileTrue(DriveCommands.dpadDriveField(drive, 180));
    // romeo.povDownLeft().whileTrue(DriveCommands.dpadDriveField(drive, 225));
    romeo.povLeft().whileTrue(DriveCommands.dpadDriveField(drive, 270));
    // romeo.povUpLeft().whileTrue(DriveCommands.dpadDriveField(drive, 315));

    // Drive constant velocity relative to robot while held
    romeo.leftBumper().whileTrue(DriveCommands.dpadDriveRobot(drive, 270)); // Crab robot left
    romeo.rightBumper().whileTrue(DriveCommands.dpadDriveRobot(drive, 90)); // Crab robot right

    // Switch to X pattern when X button is pressed
    romeo.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    romeo
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    
    //operator functions
    elevator.setDefaultCommand(
        ElevatorCommands.joystickElevator(
            elevator, () -> -juliet.getLeftY(), rightBackTop, leftBackTop));
             // back is positive, so need to invert
             // right is positive for tilt, so leave that alone
    clawIntake.setDefaultCommand(
        WristCommand.wristCommand(clawIntake, rightBack, leftBack, UpDPad, DownDPad)
    );
    //juliet.y().whileTrue(new elevatorUp(elevator, juliet, yButton)); // elevator up
    //juliet.a().whileTrue(new elevatorDown(elevator, juliet, aButton)); // elevator down
    //juliet.y().whileTrue(new ElevatorAuto3());
    //juliet.rightBumper().whileTrue(new WristForward(clawIntake, juliet, rightBack, leftBack)); // wrist forward
    //juliet.leftBumper().or(juliet.rightBumper()).whileTrue(new WristBack(clawIntake, juliet, leftBack, rightBack)); // wrist backward
    juliet.y().whileTrue(new ElevatorAuto4(elevator, clawIntake));
    juliet.x().whileTrue(new ElevatorAuto3(elevator, clawIntake));
    juliet.a().whileTrue(new ElevatorAuto2(elevator, clawIntake));
    juliet.b().whileTrue(new IntakeAuto(elevator, clawIntake));
    //juliet.x().onTrue(new RollersForward(clawIntake, juliet, xButton, bButton)); // Bring coral in
    //juliet.b().onTrue(new RollersBack(clawIntake, juliet, bButton)); // Spit coral out
// These are still tied to the triggers, but controlled through the default elevator command
//    juliet.rightTrigger().onTrue(new ElbowForward(elevator, juliet, rightBackTop)); // elbow forward
//    juliet.leftTrigger().onTrue(new ElbowBack(elevator, juliet, leftBackTop)); // elbow backwards
    juliet.start().whileTrue(new Climb(juliet, climber, startButton)); // climber
  //  juliet.start().onTrue(new ElevatorStow(elevator, juliet, startButton)); // Stow elevator / calibrate
    juliet.back().whileTrue(new ClimbBack(climber, juliet, backButton)); //climber back

    juliet.back().and(juliet.start()).onTrue(
        ElevatorCommands.logPose(elevator, "SnapPose").andThen(
        ElevatorCommands.logPose(clawIntake, "SnapPose")).andThen(
        ElevatorCommands.logPose(climber, "SnapPose"))
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
