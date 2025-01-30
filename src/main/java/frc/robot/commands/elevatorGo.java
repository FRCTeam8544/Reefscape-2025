package frc.robot.commands;

import frc.robot.subsystems.elevator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class elevatorGo extends Command{
    XboxController operator;
    elevator elevator; 
        public elevatorGo(){}


    @Override
    public void initialize(){}
    @Override 
    public void execute(){
        if(!elevator.upStop.getAsBoolean() && operator.getAButtonPressed()){
            elevator.climberMove(true);}
        if(!elevator.downStop.getAsBoolean() && operator.getBButtonPressed()){
            elevator.climberMove(false);}
    }
    @Override
    public boolean isFinished(){}
}
