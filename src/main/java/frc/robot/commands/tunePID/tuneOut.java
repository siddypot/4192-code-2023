package frc.robot.commands.tunePID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Elevator;

public class tuneOut extends CommandBase{
    
    PIDController pid = new PIDController(1, 0, 0);
    //private final double value;

    //private final Elevator e;
    //public tuneOut(Elevator e, double b){
      //  value = b;
        //this.e=e;

    //}

    @Override
    public void execute(){        



        //e.moveElbow(value);

    }

    //@Override
    //public boolean isFinished(){

        //return 
    //}


    
}
