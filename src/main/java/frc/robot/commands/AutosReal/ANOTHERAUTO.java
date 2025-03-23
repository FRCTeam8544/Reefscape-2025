// package frc.robot.commands.AutosReal;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Elevator.IntakeAuto;
// import frc.robot.subsystems.ClawIntake;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.drive.Drive;

// public class ANOTHERAUTO extends SequentialCommandGroup{

//     public ANOTHERAUTO(Drive drive, Elevator elevator, ClawIntake clawIntake){
//         ChassisSpeeds chassisSpeeds1 = new ChassisSpeeds();
//         ChassisSpeeds chassisSpeeds2 = new ChassisSpeeds();
//         ChassisSpeeds chassisSpeeds3 = new ChassisSpeeds();

//         chassisSpeeds1.vxMetersPerSecond = 1;
//         chassisSpeeds1.vyMetersPerSecond = 0;

//         chassisSpeeds2.vxMetersPerSecond = 0.3;
//         chassisSpeeds2.vyMetersPerSecond = 0;

//         chassisSpeeds3.vxMetersPerSecond = 0.3;
//         chassisSpeeds3.vyMetersPerSecond = 0;
//         addCommands(
//            new SequentialCommandGroup(
//                 new freakyahdriveauto(drive, chassisSpeeds1),
//                 new ElevatorPose4(elevator),
//                 new freakyahprepareauto(elevator, clawIntake),
//                 new freakyahdriveauto(drive, chassisSpeeds2),
//                 new freakyahplaceauto(elevator, clawIntake),
//                 new IntakeAuto(elevator, clawIntake)
//         )  
//         );
       
//     }
    
// }
