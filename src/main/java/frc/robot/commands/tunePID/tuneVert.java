package frc.robot.commands.tunePID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Elevator;

public class tuneVert extends CommandBase{
    
    PIDController pid = new PIDController(0, 0, 0);

    private final Elevator e;
    public tuneVert(Elevator e){

        this.e=e;

    }

    @Override
    public void execute(){

        double middlevalue = 40;
        
        double offset = e.leftEle.getEncoder().getPosition();

        e.raiseEle(pid.calculate(offset,middlevalue));

    }


    
}
