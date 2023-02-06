package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class autoBalance extends CommandBase{
    private final Swerve swerve;
    private final PIDController pid = new PIDController(0, 0, 0);

    public autoBalance(Swerve s){
        swerve = s;
    }

    @Override
    public void execute(){

    }
    
}


/* reminder:
    
    detect the direction of the robot (NWSE)

    get pitch (if NS (relative to charge station))
    get roll (if WE)

      pid it up 

      fix the crappy charge station with ian or aravind or anik

      tune it up 
     */