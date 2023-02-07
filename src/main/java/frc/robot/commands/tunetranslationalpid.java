package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class tunetranslationalpid extends CommandBase{
    private final Swerve swerve;
    private final PIDController pid = new PIDController(.5, 0, 0); //to be tuned

    public tunetranslationalpid(Swerve s){

        swerve = s;
    }

    @Override
    public void execute(){

        double offset = swerve.getPosX(); //not right??

        swerve.drive(new Translation2d(pid.calculate(offset, 0),0),0,true, true);
    

    }
    
}