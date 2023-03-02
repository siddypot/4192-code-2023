package frc.robot;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick opps = new Joystick(1);



    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;

    private final int elevatorUpAxis = 1; //neg
    private final int elbowOutAxis = 0;
    private final int wristAxis = 5;


    private final int spinCWtrigga = 3;
    private final int spinCCWtrigga = 2;




    /* Driver Buttons */                                                                        

    private final JoystickButton zeroGyro = new JoystickButton(driver, 2);
    private final JoystickButton robotCentric = new JoystickButton(driver, 1);
    //private final JoystickButton align = new JoystickButton(driver, 5);
    private final JoystickButton xWheel = new JoystickButton(driver, 3);
    private final JoystickButton autoBalance = new JoystickButton(driver, 4);
    //private final JoystickButton tunePid = new JoystickButton(driver, 6);
    private final JoystickButton opensesame = new JoystickButton(opps, 5);
    private final JoystickButton closesesame = new JoystickButton(opps, 6);

    

    private final JoystickButton eleUp = new JoystickButton(opps,1);
    private final JoystickButton sdi = new JoystickButton(opps, 2);
    private final JoystickButton wristUp = new JoystickButton(opps, 3);
    private final JoystickButton wristDawn = new JoystickButton(opps, 4);
    private final JoystickButton elbowout = new JoystickButton(opps, 5);
    private final JoystickButton elbowin = new JoystickButton(opps, 6);
   
    private final Trigger intake = new Trigger(() -> opps.getRawAxis(spinCWtrigga) > .2);
    private final Trigger outtake = new Trigger(() -> opps.getRawAxis(spinCCWtrigga) > .2);




    /* Subsystems */
    private final Limelight limelight = new Limelight(getAllianceColor());
    private final Swerve swerve = new Swerve(limelight);
    private final Elevator elevator = new Elevator();


    /* Commands */
    private final autoAlign align1 = new autoAlign(swerve);
    private final xWheels xwheel = new xWheels(swerve);
    private final autoBalance ab = new autoBalance(swerve);
    private final tunetranslationalpid tunePID = new tunetranslationalpid(swerve);

    private final elbowIn extend = new elbowIn(elevator, true);
    private final elbowOut retract = new elbowOut(elevator, true);
    private final spinnything spindaGrippa = new spinnything(elevator,() -> opps.getRawAxis(spinCWtrigga));
    private final spinnything backDaGrippa = new spinnything(elevator,() -> -opps.getRawAxis(spinCCWtrigga));

    




    public RobotContainer() {
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        elevator.setDefaultCommand(new raiseTheEle(
                elevator,
                () -> -opps.getRawAxis(elevatorUpAxis),
                () -> opps.getRawAxis(elbowOutAxis),
                () -> -opps.getRawAxis(wristAxis)));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        
        zeroGyro.onTrue(new InstantCommand(() -> swerve.resetGyro()));
        //align.whileTrue(align1);
        xWheel.whileTrue(xwheel);
        autoBalance.whileTrue(ab);
       // tunePid.whileTrue(tunePID);


        outtake.whileTrue(backDaGrippa);
        intake.whileTrue(spindaGrippa);




        closesesame.whileTrue(new InstantCommand(() -> elevator.Open()) );
        opensesame.whileTrue(new InstantCommand(() -> elevator.Close()));


        
        
    }
    public Command getAutonomousCommand() {

        PIDController pidstuff = new PIDController(.15, 0, .000005);
        PIDController angularPID = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI ,Constants.Swerve.angleKD);
        angularPID.enableContinuousInput(-Math.PI, Math.PI);
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Auton", Constants.Swerve.maxSpeed, Constants.Swerve.maxAccel);
        ProfiledPIDController thetaPid = new ProfiledPIDController(.15,0, .000005, Constants.AutoConstants.kThetaControllerConstraints);
        HolonomicDriveController driveController = new HolonomicDriveController(pidstuff, pidstuff, thetaPid);
        Trajectory newTrajectory = trajectory.relativeTo(swerve.getPose());
        PPSwerveControllerCommand autonCommand0 = new PPSwerveControllerCommand (
            trajectory, 
            swerve::getPose,
            Constants.Swerve.swerveKinematics, 
            pidstuff, 
            pidstuff,  
            angularPID,
            swerve::setModuleStates,
            swerve
        );


        SwerveControllerCommand autonCommand1 = new SwerveControllerCommand(
            newTrajectory,
             swerve::getPose,
              Constants.Swerve.swerveKinematics,
               driveController,
                swerve::setModuleStates,
                swerve);

        SequentialCommandGroup finalAutonCommand = new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetGyro()),
        autonCommand1,
        new xWheels(swerve),
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d()))
        );
        return finalAutonCommand;
    }


    public boolean getAllianceColor(){ //only if blue

        String color = DriverStation.getAlliance().toString();
        if(color.equals("Blue")) return true;
        return false;
    }
}

