package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class elbowIn extends CommandBase{

    private final Elevator e;
    private boolean in;
    public elbowIn(Elevator e, boolean in){
        this.e = e;
        this.in = in;
    }


    @Override
    public void execute(){

        //true to go out 
        //false to move back 
        
        e.retractElbow();

    }
    @Override
    public void end(boolean interupt){
        e.idleElbow();
    }
}
