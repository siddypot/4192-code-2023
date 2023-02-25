package frc.robot.commands;

import javax.lang.model.element.Element;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class spinnything extends CommandBase{

    private final Elevator e;
    private double power;

    public spinnything(Elevator e, double power){
        this.e = e;
        this.power = power;
    }


    @Override
    public void execute(){
        e.intakePower(power);

    }

    @Override
    public void end(boolean asda){
        e.idleIntake();
    }


    
}
