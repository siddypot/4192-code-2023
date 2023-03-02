package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class doaflip extends CommandBase{

    boolean b;
    Elevator elevation;

    public doaflip(Elevator e, boolean b){

        elevation = e;
        this.b = b;
        

    }

    @Override
    public void execute(){
        if(!elevation.getIfOpen()){ elevation.Open();}        
        
        else{ elevation.Close();}
    
    }
    
}
