package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class raiseTheEle extends CommandBase {

    
    private final Elevator e;
    double power, powa;
    private int upElevatorPos = 0;
    private int downElevatorPos = 0;
    
    public raiseTheEle(Elevator ele, double powerForEle, double powerForElbow){
        this.power = powerForEle;
        this.powa = powerForElbow;
        e = ele;
    }

    
    @Override
    public void execute(){

        double elevatorPower = MathUtil.applyDeadband(power, Constants.stickDeadband);
        double elbowPower = MathUtil.applyDeadband(powa, Constants.stickDeadband);


        e.raiseElePower(elevatorPower);
        e.doElbowThing(elbowPower);

        
        
    }


    @Override
    public void end(boolean interupt){
        e.zeroEle();
    }
    
}
