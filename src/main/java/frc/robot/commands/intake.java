package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class intake extends CommandBase {

    
    private final Elevator e;

    public intake(Elevator ele){
        e = ele;
    }

    
    @Override
    public void execute(){

        e.intakePower();

        
        
    }


    @Override
    public void end(boolean interupt){
        e.intakeIdle();
    }
}
