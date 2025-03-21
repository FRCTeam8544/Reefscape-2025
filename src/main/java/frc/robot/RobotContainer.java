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

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.WristForward;
import frc.robot.commands.AutonomousAuto.AutoScore;
import frc.robot.commands.AutonomousAuto.IntakeAuto;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbBack;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElbowBack;
import frc.robot.commands.RollersForward;
import frc.robot.commands.RollersBack;
import frc.robot.commands.ElbowForward;
import frc.robot.commands.WristBack;
import frc.robot.commands.Elevator.elevatorDown;
import frc.robot.commands.Elevator.elevatorUp;
import frc.robot.commands.Elevator.ElevatorCommands;
import frc.robot.commands.Elevator.ElevatorStow;
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

import org.json.simple.parser.ParseException;
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
  private int bestTargetID = -1;
  private final VisionIOPhotonVision leftCamera =  new VisionIOPhotonVision(leftChassisApriltag, robotToCamera0);
  private final VisionIOPhotonVision rightCamera =  new VisionIOPhotonVision(rightChassisApriltag, robotToCamera1);
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

  // PathPlanner
  private final PathConstraints constraints = new PathConstraints(
    3.0, 4.0,
    Units.degreesToRadians(540), Units.degreesToRadians(720));

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    //named commands for pathplanner
    NamedCommands.registerCommand("AutoScore4", new AutoScore(elevator, clawIntake));
    NamedCommands.registerCommand("IntakePose", new IntakeAuto(elevator, clawIntake));

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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                leftCamera, rightCamera);
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

    // Switch to X pattern when Back button is pressed
    romeo.back().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Start button is pressed
    romeo
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    romeo
        .b().onTrue(runCenterApproach());
    romeo
        .a().onTrue(runRightApproach());
    romeo
        .x().onTrue(runLeftApproach());
    romeo
        .y().onTrue(runSourceApproach());
    

    //operator functions
    elevator.setDefaultCommand(
        ElevatorCommands.joystickElevator(
            elevator, () -> -juliet.getLeftY(), () -> juliet.getLeftX()));
             // back is positive, so need to invert
             // right is positive for tilt, so leave that alone

    juliet.y().onTrue(new elevatorUp(elevator, juliet, yButton)); // elevator up
    juliet.a().onTrue(new elevatorDown(elevator, juliet, aButton)); // elevator down
    juliet.rightBumper().onTrue(new WristForward(clawIntake, juliet, rightBack)); // wrist forward
    juliet.leftBumper().onTrue(new WristBack(clawIntake, juliet, leftBack)); // wrist backward
    juliet.x().onTrue(new RollersForward(clawIntake, juliet, xButton, bButton)); // Bring coral in
    juliet.b().onTrue(new RollersBack(clawIntake, juliet, bButton)); // Spit coral out
    juliet.rightTrigger().onTrue(new ElbowForward(elevator, juliet, rightBackTop)); // elbow forward
    juliet.leftTrigger().onTrue(new ElbowBack(elevator, juliet, leftBackTop)); // elbow backwards
    juliet.start().onTrue(new Climb(juliet, climber, startButton)); // climber
  //  juliet.start().onTrue(new ElevatorStow(elevator, juliet, startButton)); // Stow elevator / calibrate
    juliet.back().onTrue(new ClimbBack(climber, juliet, backButton)); //climber back

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
  //all approach commands assume blue tags are 1-6 and red tags are 7-12
  public Command runCenterApproach() {
    PathPlannerPath path = null;
    PathPlannerPath center1 = null;
    try {
        center1 = PathPlannerPath.fromPathFile("Center 1");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center2 = null;
    try {
        center2 = PathPlannerPath.fromPathFile("Center 2");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center3 = null;
    try {
        center3 = PathPlannerPath.fromPathFile("Center 3");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center4 = null;
    try {
        center4 = PathPlannerPath.fromPathFile("Center 4");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center5 = null;
    try {
        center5 = PathPlannerPath.fromPathFile("Center 5");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center6 = null;
    try {
        center6 = PathPlannerPath.fromPathFile("Center 6");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center7 = null;
    try {
        center7 = PathPlannerPath.fromPathFile("Center 7");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center8 = null;
    try {
        center8 = PathPlannerPath.fromPathFile("Center 8");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center9 = null;
    try {
        center9 = PathPlannerPath.fromPathFile("Center 9");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center10 = null;
    try {
        center10 = PathPlannerPath.fromPathFile("Center 10");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center11 = null;
    try {
        center11 = PathPlannerPath.fromPathFile("Center 11");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath center12 = null;
    try {
        center12 = PathPlannerPath.fromPathFile("Center 12");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }

    switch (findBestTargetID()) {
        case 19:
            path = center1;
        case 18: 
            path = center2;
        case 17:
            path = center3;
        case 22:
            path = center4;
        case 21:
            path = center5;
        case 20:
            path = center6;
        case 6:
            path = center7;
        case 7:
            path = center8;
        case 8:
            path = center9;
        case 9:
            path = center10;
        case 10:
            path = center11;
        case 11:
            path = center12;

    }
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    return pathfindingCommand;
  }

  public Command runLeftApproach() {
    PathPlannerPath path = null;
    PathPlannerPath left1 = null;
    try {
        left1 = PathPlannerPath.fromPathFile("Left 1");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left2 = null;
    try {
        left2 = PathPlannerPath.fromPathFile("Left 2");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left3 = null;
    try {
        left3 = PathPlannerPath.fromPathFile("Left 3");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left4 = null;
    try {
        left4 = PathPlannerPath.fromPathFile("Left 4");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left5 = null;
    try {
        left5 = PathPlannerPath.fromPathFile("Left 5");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left6 = null;
    try {
        left6 = PathPlannerPath.fromPathFile("Left 6");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left7 = null;
    try {
        left7 = PathPlannerPath.fromPathFile("Left 7");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left8 = null;
    try {
        left8 = PathPlannerPath.fromPathFile("Left 8");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left9 = null;
    try {
        left9 = PathPlannerPath.fromPathFile("Left 9");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left10 = null;
    try {
        left10 = PathPlannerPath.fromPathFile("Left 10");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left11 = null;
    try {
        left11 = PathPlannerPath.fromPathFile("Left 11");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath left12 = null;
    try {
        left12 = PathPlannerPath.fromPathFile("Left 12");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }

    switch (findBestTargetID()) {
        case 19:
            path = left1;
        case 18: 
            path = left2;
        case 17:
            path = left3;
        case 22:
            path = left4;
        case 21:
            path = left5;
        case 20:
            path = left6;
        case 6:
            path = left7;
        case 7:
            path = left8;
        case 8:
            path = left9;
        case 9:
            path = left10;
        case 10:
            path = left11;
        case 11:
            path = left12;
    }
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    return pathfindingCommand;
  }

  public Command runRightApproach() {
    PathPlannerPath path = null;
    PathPlannerPath right1 = null;
    try {
        right1 = PathPlannerPath.fromPathFile("Right 1");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right2 = null;
    try {
        right2 = PathPlannerPath.fromPathFile("Right 2");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right3 = null;
    try {
        right3 = PathPlannerPath.fromPathFile("Right 3");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right4 = null;
    try {
        right4 = PathPlannerPath.fromPathFile("Right 4");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right5 = null;
    try {
        right5 = PathPlannerPath.fromPathFile("Right 5");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right6 = null;
    try {
        right6 = PathPlannerPath.fromPathFile("Right 6");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right7 = null;
    try {
        right7 = PathPlannerPath.fromPathFile("Right 7");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right8 = null;
    try {
        right8 = PathPlannerPath.fromPathFile("Right 8");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right9 = null;
    try {
        right9 = PathPlannerPath.fromPathFile("Right 9");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right10 = null;
    try {
        right10 = PathPlannerPath.fromPathFile("Right 10");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right11 = null;
    try {
        right11 = PathPlannerPath.fromPathFile("Right 11");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath right12 = null;
    try {
        right12 = PathPlannerPath.fromPathFile("Right 12");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }

    switch (findBestTargetID()) {
        case 19:
            path = right1;
        case 18: 
            path = right2;
        case 17:
            path = right3;
        case 22:
            path = right4;
        case 21:
            path = right5;
        case 20:
            path = right6;
        case 6:
            path = right7;
        case 7:
            path = right8;
        case 8:
            path = right9;
        case 9:
            path = right10;
        case 10:
            path = right11;
        case 11:
            path = right12;
    }
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    return pathfindingCommand;
  }

  public Command runSourceApproach() {
    PathPlannerPath path = null;
    PathPlannerPath blue1 = null;
    try {
        blue1 = PathPlannerPath.fromPathFile("Blue Source 1");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath blue2 = null;
    try {
        blue2 = PathPlannerPath.fromPathFile("Blue Source 2");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath red3 = null;
    try {
        red3 = PathPlannerPath.fromPathFile("Red Source 1");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }
    PathPlannerPath red4 = null;
    try {
        red4 = PathPlannerPath.fromPathFile("Red Source 2");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
    }

    switch (findBestTargetID()) {
        case 13:
            path = blue1;
        case 12: 
            path = blue2;
        case 1:
            path = red3;
        case 2:
            path = red4;
    }
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
    return pathfindingCommand;
}


  private int findBestTargetID() { //compares both camera's best tags ambiguitity
    int leftBest = leftCamera.getBestTargetID();
    int rightBest = rightCamera.getBestTargetID();

    if(leftBest == rightBest)
        return leftBest;
    else if (leftCamera.getBestTargetAbiguitity() < rightCamera.getBestTargetAbiguitity())
        return leftBest; //has lower ambiguitity, so probably more correct
    else
        return rightBest;
  }
}
