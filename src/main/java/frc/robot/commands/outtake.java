package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class outtake extends CommandBase {

    
    private final Elevator e;

    public outtake(Elevator ele){
        e = ele;
    }

    
    @Override
    public void execute(){

        e.outtakePower();


    }


    @Override
    public void end(boolean interupt){
        e.outtakeIdle();
    }
    
}