package frc.robot.subsystems;

import com.ctre.phoenix.led.RgbFadeAnimation;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{


    private final CANSparkMax lGrippa;
    private final CANSparkMax rGrippa;


    public final CANSparkMax leftEle;
    public final CANSparkMax rightEle;

    public final CANSparkMax elbow;
    private final CANSparkMax wrist;

    private final DoubleSolenoid inSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 8, 9);


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


        
    }


    public void motorSetup(CANSparkMax c){
        c.setSmartCurrentLimit(30);
        c.restoreFactoryDefaults();
        c.setIdleMode(IdleMode.kBrake);
    }
    public void raiseRight(double goalHeight){

        rightEle.set(PID.calculate(rightEle.getEncoder().getPosition(), goalHeight));

    }

    public void raiseLeft(double height){
        leftEle.set(PID.calculate(leftEle.getEncoder().getPosition(), height));

    }

    public void raiseElePower(double speed){

        leftEle.set(.8 * speed);
        rightEle.set(.8 * speed);

    }


    public void intakePower(double playa){
        lGrippa.set(playa);
        rGrippa.set(playa);
    }

    public void idleIntake(){
        lGrippa.set(0.01);
        rGrippa.set(0.01);
    }

    public void Close(){
        inSolenoid.set(Value.kReverse);
    }

    public void Open(){
        inSolenoid.set(Value.kForward);
    }

    public boolean getIfOpen(){
        return (inSolenoid.get().equals(Value.kForward));
    }


    public void moveElbow(double position){
        
        elbow.set(elbowPID.calculate(elbow.getEncoder().getPosition(), position));

    }



public void zeroEle(){

        leftEle.set(0.095);
        rightEle.set(0.095);
    }


    public void idleWrist(){
        wrist.set(0);
    }

    
    public void flickofdawrist(double power){
        wrist.set(power * .8);
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
