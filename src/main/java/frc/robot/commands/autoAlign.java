package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class autoAlign extends CommandBase{

    PIDController pid = new PIDController(.15, 0, .000005);

    private Swerve swerve;
    public autoAlign(Swerve s){
        swerve = s;
    }
    @Override
    public void execute(){
        // stop moving the robot if it's within +- 1 degree of 
    
        double offset = swerve.getDegreesYaw(); //offset
        swerve.drive(new Translation2d(0 ,0), 
        
        -pid.calculate(offset,0),
        
        true, true);
    }

}
