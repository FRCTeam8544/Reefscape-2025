package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawIntake;

public class WristCommand {
    public static Command wristCommand(
        ClawIntake clawIntake, Trigger rightBack, Trigger leftBack, Trigger xButton, Trigger bButton){
            return Commands.run(() -> {
                // if (leftBack.getAsBoolean() && !clawIntake.wristBackwardStop.getAsBoolean()) {
                //     clawIntake.wristTurnBack(true);
                // }else if (rightBack.getAsBoolean() && !clawIntake.wristForwardStop.getAsBoolean()){
                //     clawIntake.wristTurn(true);
                // }
                // else {clawIntake.wristTurn(false);}
                if (leftBack.getAsBoolean() ^ rightBack.getAsBoolean()){
                    double fortnite = leftBack.getAsBoolean()? 0.05 : -0.05;
                    clawIntake.setPositionSetPoint(fortnite + clawIntake.getPos());
                }else{
                    clawIntake.setPositionSetPoint(clawIntake.getPos());
                }

                if ( (xButton.getAsBoolean() && !clawIntake.hasCoral()) ||
                    (xButton.getAsBoolean() && bButton.getAsBoolean()) ) {
                    clawIntake.rollerRoll(true);
                } else if (bButton.getAsBoolean()){
                    clawIntake.rollerRollBack(true);
                } else {
                    clawIntake.rollerRoll(false);
                }
            }, clawIntake);
        }
}
