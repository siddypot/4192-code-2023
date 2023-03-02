package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class autoBalance extends CommandBase{
    private final Swerve swerve;
    private final PIDController pid = new PIDController(.060, 0, .005); //to be tuned

    public autoBalance(Swerve s){

        swerve = s;
        swerve.resetGyro();
    }

    @Override
    public void execute(){

        double offset = swerve.getRoll() - 0;

        swerve.drive(
            new Translation2d(pid.calculate(offset, 0),0),
            0,true, true);
    

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