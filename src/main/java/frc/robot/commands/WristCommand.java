package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawIntake;
import frc.robot.subsystems.LEDs;

public class WristCommand {
    public static Command wristCommand(
        LEDs leds, ClawIntake clawIntake, Trigger rightBack, Trigger leftBack, Trigger xButton, Trigger bButton){
            return Commands.run(() -> {
                // if (leftBack.getAsBoolean() && !clawIntake.wristBackwardStop.getAsBoolean()) {
                //     clawIntake.wristTurnBack(true);
                // }else if (rightBack.getAsBoolean() && !clawIntake.wristForwardStop.getAsBoolean()){
                //     clawIntake.wristTurn(true);
                // }
                // else {clawIntake.wristTurn(false);}
                if (leftBack.getAsBoolean() ^ rightBack.getAsBoolean()){
                    double fortnite = leftBack.getAsBoolean()? 0.1 : -0.1;
                    clawIntake.setPositionSetPoint(fortnite + clawIntake.getPos());
                }else{
                    clawIntake.setPositionSetPoint(clawIntake.getPos());
                /*if (leftBack.getAsBoolean() && !clawIntake.wristBackwardStop.getAsBoolean()) {
                    clawIntake.wristTurnBack(true);
                }else if (rightBack.getAsBoolean() && !clawIntake.wristForwardStop.getAsBoolean()){
                    clawIntake.wristTurn(true);
                }
                else {clawIntake.wristTurn(false);}  */


                if ( xButton.getAsBoolean() ){
                    if( !clawIntake.hasCoral())  {
                    clawIntake.rollerRoll(true); 
                    leds.clawIntake();  }
                    else{leds.coralAquired(); clawIntake.rollerRoll(false);}
                    
                } else if (bButton.getAsBoolean()){
                    clawIntake.rollerRollBack(true);
                    leds.violet();
                } else {
                    clawIntake.rollerRoll(false);
                    leds.violet();
                }
//
                if (clawIntake.hasCoral()) {
                    leds.coralAquired();
                }
                
            //
            }}, clawIntake);
        }
}

