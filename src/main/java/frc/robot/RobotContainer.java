package frc.robot;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */                                                                        

    private final JoystickButton zeroGyro = new JoystickButton(driver, 2);
    private final JoystickButton robotCentric = new JoystickButton(driver, 1);
    private final JoystickButton align = new JoystickButton(driver, 5);
    private final JoystickButton xWheel = new JoystickButton(driver, 3);
    private final JoystickButton autoBalance = new JoystickButton(driver, 4);



    /* Subsystems */

    private final Swerve swerve = new Swerve();


    private final autoAlign align1 = new autoAlign(swerve);
    private final xWheels xwheel = new xWheels(swerve);
    private final autoBalance ab = new autoBalance(swerve);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.resetGyro()));
        align.whileTrue(align1);
        xWheel.whileTrue(xwheel);
        autoBalance.whileTrue(ab);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {



        PIDController pidstuff = new PIDController(0,0,0);
        PIDController angularPID = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI ,Constants.Swerve.angleKD);
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("jackk", Constants.Swerve.maxSpeed, Constants.Swerve.maxAccel);
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("balls", Constants.Swerve.maxSpeed, Constants.Swerve.maxAccel);

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

        SequentialCommandGroup finalAutonCommand = new SequentialCommandGroup(

        new InstantCommand(() -> swerve.resetGyro()),

        autonCommand0,
        
        new xWheels(swerve),
  
        new InstantCommand(() -> swerve.resetOdometry(new Pose2d()))
        
        );
        
        return finalAutonCommand;
    }
}

