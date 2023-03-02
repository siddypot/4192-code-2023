package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class raiseTheEle extends CommandBase {

    
    private final Elevator e;
    DoubleSupplier power, powa, pow;
    private int upElevatorPos = 0;
    private int downElevatorPos = 0;
    
    public raiseTheEle(Elevator ele, DoubleSupplier powerForEle, DoubleSupplier powerForElbow, DoubleSupplier elbowJohnson){
        this.power = powerForEle;
        this.powa = powerForElbow;
        e = ele;
        this.pow = elbowJohnson;
        addRequirements(e);
    }

    
    @Override
    public void execute(){

        double elevatorPower =  MathUtil.applyDeadband(power.getAsDouble(), Constants.stickDeadband);
        double elbowPower = MathUtil.applyDeadband(powa.getAsDouble(), Constants.stickDeadband);
        double thunderClan = MathUtil.applyDeadband(pow.getAsDouble(), Constants.stickDeadband);


        e.raiseElePower(elevatorPower);
        e.doElbowThing(elbowPower);
        e.flickofdawrist(thunderClan);

        
        
    }


    @Override
    public void end(boolean interupt){
    }
    
}
