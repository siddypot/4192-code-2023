package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.lang.model.element.Element;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class spinnything extends CommandBase{

    private final Elevator e;
    private DoubleSupplier power;

    public spinnything(Elevator e, DoubleSupplier power){
        this.e = e;
        this.power = power;
    }


    @Override
    public void execute(){
        e.intakePower( power.getAsDouble());

    }

    @Override
    public void end(boolean asda){
        e.idleIntake();
    }


    
}
