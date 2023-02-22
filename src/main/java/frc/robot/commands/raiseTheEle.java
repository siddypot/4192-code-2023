package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class raiseTheEle extends CommandBase {

    
    private final Elevator e;
    private boolean up = false;

    private int upElevatorPos = 0;
    private int downElevatorPos = 0;
    public raiseTheEle(Elevator ele, boolean up){
        this.up = up;
        e = ele;
    }

    public void up(){
        e.raiseEle(upElevatorPos);
    }
    
    public void down(){
        e.raiseEle(downElevatorPos);
    }
    
    @Override
    public void execute(){
        if(up) {e.raiseElePower();}

        else e.raiseElePower();
    }

    
}
