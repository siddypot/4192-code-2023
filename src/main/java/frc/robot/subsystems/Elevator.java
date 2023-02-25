package frc.robot.subsystems;

import com.ctre.phoenix.led.RgbFadeAnimation;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{


    private final CANSparkMax lGrippa;
    private final CANSparkMax rGrippa;


    public final CANSparkMax leftEle;
    public final CANSparkMax rightEle;

    private final CANSparkMax elbow;
    private final CANSparkMax wrist;


    PIDController PID = new PIDController(1, 0, 0);
    

    PIDController elbowPID = new PIDController(1, 0, 0);


    public Elevator(){

        elbow = new CANSparkMax(3, MotorType.kBrushless);
        wrist = new CANSparkMax(4,MotorType.kBrushless);


        leftEle = new CANSparkMax(1, MotorType.kBrushless);
        rightEle = new CANSparkMax(2, MotorType.kBrushless);


        rGrippa = new CANSparkMax(5, MotorType.kBrushless);
        lGrippa = new CANSparkMax(6, MotorType.kBrushless);





        


        motorSetup(rightEle);
        motorSetup(leftEle);

        motorSetup(elbow);
        motorSetup(wrist);

        motorSetup(lGrippa);
        motorSetup(rGrippa);

        rGrippa.setSmartCurrentLimit(10);
        lGrippa.setSmartCurrentLimit(10);


        leftEle.setInverted(true);
        rightEle.setInverted(false);

        
        lGrippa.setInverted(true);

        wrist.setSmartCurrentLimit(15);

        
    }


    public void motorSetup(CANSparkMax c){
        c.setSmartCurrentLimit(30);
        c.restoreFactoryDefaults();
        c.setIdleMode(IdleMode.kBrake);
    }
    public void raiseEle(double goalHeight){

        rightEle.set(PID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
        leftEle.set(PID.calculate(leftEle.getEncoder().getPosition(), goalHeight));

    }

    public void raiseElePower(double speed){

        leftEle.set(.4 * speed);
        rightEle.set(.4 * speed);

    }

    public void DownraiseElePower(){

        leftEle.set(-.2);
        rightEle.set(-.2);

    }

    public void intakePower(){
        lGrippa.set(-1);
        rGrippa.set(-1);
    }

    public void intakeIdle(){
        lGrippa.set(0);
        rGrippa.set(0);
    }

    public void outtakePower(){
        lGrippa.set(.5);
        rGrippa.set(.5);
    }

    public void outtakeIdle(){
        lGrippa.set(0);
        rGrippa.set(0);
    }

    public void zeroEle(){

        leftEle.set(0.01);
        rightEle.set(0.01);
    }

    public void movewrist(){
        wrist.set(.6);
    }
    public void movewrisNt(){
        wrist.set(-.6);
    }

    public void idleWrist(){
        wrist.set(.0);
    }

    public void wearescrewed(){} // method to set the elevator to one motor mode and the other becomes follower
    

    public void wearescrewedbutstilltryandwork(){

        leftEle.follow(rightEle);


        double goalHeight = 0;

        leftEle.set(PID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
        rightEle.set(PID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
    }



    public void moveElbow(boolean direction){

        double in = 0; //change
        double out = 0; //change


        //if direction is true then go out 


        if(direction){

            if(elbow.getEncoder().getPosition() <= (out + 50)) elbowPID.calculate(elbow.getEncoder().getPosition(), out);

        }

        else {

            if(elbow.getEncoder().getPosition() > (in + 50)) elbowPID.calculate(elbow.getEncoder().getPosition(), in);

        }

    }

    public void extendElbow(){
        elbow.set(.2);
    }

    public void doElbowThing(double speed){
        elbow.set(speed * .3);
    }

    public void retractElbow(){
        elbow.set(-.17);
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("rightmoroPos", rightEle.getEncoder().getPosition());
        SmartDashboard.putNumber("leftMotorPos", leftEle.getEncoder().getPosition());
        SmartDashboard.putNumber("elbow", elbow.getEncoder().getPosition());

    }

    public void idleElbow(){
        elbow.set(.01);
    }

    
    
}
