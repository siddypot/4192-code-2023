package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class elbowOut extends CommandBase{

    private final Elevator e;
    private boolean out;
    public elbowOut(Elevator e, boolean out){
        this.e = e;
        this.out = out;
    }


    @Override
    public void execute(){

        //true to go out 
        //false to move back 
        
        e.extendElbow();

    }
    @Override
    public void end(boolean interupt){
        e.idleElbow();
    }
}
