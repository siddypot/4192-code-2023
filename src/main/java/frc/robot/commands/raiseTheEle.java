package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class raiseTheEle extends CommandBase {

    
    private final Elevator e;
    private boolean up = false;

    private int upElevatorPos = 0;
    private int downElevatorPos = 0;
    public raiseTheEle(Elevator ele){
        this.up = up;
        e = ele;
    }

    
    @Override
    public void execute(){

        e.raiseElePower();

        
        
    }


    @Override
    public void end(boolean interupt){
        e.zeroEle();
    }
    
}
