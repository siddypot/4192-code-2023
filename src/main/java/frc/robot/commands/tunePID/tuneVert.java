package frc.robot.commands.tunePID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Elevator;

public class tuneVert extends CommandBase{
    
    PIDController pid = new PIDController(1, 0, 0);

    private final double value;

    private final Elevator e;
    public tuneVert(Elevator e, double value){

        this.value = value;

        this.e=e;

    }

    @Override
    public void execute(){

    
        e.raiseLeft(value);
        e.raiseRight(value);

    }

    @Override
    public boolean isFinished(){

        return (
            e.leftEle.getEncoder().getPosition() == value && 
            e.rightEle.getEncoder().getPosition() == value);

    }


    
}
