package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class xWheels extends CommandBase{ //NO WORK NO POINT IN FIXING
    
    private final Swerve swerve;
    private final Timer t;

    public xWheels(Swerve s){
        t = new Timer();
        t.reset();
        swerve = s;
    }

    @Override
    public void execute(){
        SwerveModuleState[] state = {
            new SwerveModuleState(0.1, new Rotation2d(3*Math.PI /4)),
            new SwerveModuleState(0.1, new Rotation2d(Math.PI /4)),
            new SwerveModuleState(0.1, new Rotation2d(5*Math.PI /4)),
            new SwerveModuleState(0.1, new Rotation2d(7*Math.PI /4))
            };
        swerve.setModuleStates(state);
        t.start();
    }

    @Override
    public void end(boolean interrupted){
        t.stop();
        t.reset();

        SwerveModuleState[] state = {
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0))
            };
        swerve.setModuleStates(state);

    }

    @Override
    public boolean isFinished(){
        return t.get() > .5;
    }
    

        
}
