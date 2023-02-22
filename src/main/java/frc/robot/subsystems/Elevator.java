package frc.robot.subsystems;

import com.ctre.phoenix.led.RgbFadeAnimation;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {


    private final CANSparkMax lGrippa;
    private final CANSparkMax rGrippa;


    private final CANSparkMax leftEle;
    private final CANSparkMax rightEle;

    private final CANSparkMax elbow;
    private final CANSparkMax wrist;


    PIDController leftPID = new PIDController(1, 0, 0);
    PIDController rightPID = new PIDController(1, 0, 0);

    PIDController elbowPID = new PIDController(1, 0, 0);


    public Elevator(){

        elbow = new CANSparkMax(3, MotorType.kBrushless);
        wrist = new CANSparkMax(4, MotorType.kBrushless);


        leftEle = new CANSparkMax(1, MotorType.kBrushless);
        rightEle = new CANSparkMax(2, MotorType.kBrushless);


        rGrippa = new CANSparkMax(5, MotorType.kBrushless);
        lGrippa = new CANSparkMax(6, MotorType.kBrushless);





        leftEle.setInverted(true);
        rightEle.setInverted(false);


        motorSetup(rightEle);
        motorSetup(leftEle);

        motorSetup(elbow);
        motorSetup(wrist);

        motorSetup(lGrippa);
        motorSetup(rGrippa);

        rGrippa.setSmartCurrentLimit(10);
        lGrippa.setSmartCurrentLimit(10);



        
    }


    public void motorSetup(CANSparkMax c){
        c.setSmartCurrentLimit(20);
        c.restoreFactoryDefaults();
    }
    public void raiseEle(double goalHeight){

        rightEle.set(rightPID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
        leftEle.set(leftPID.calculate(leftEle.getEncoder().getPosition(), goalHeight));

    }

    public void raiseElePower(){

        leftEle.set(.1);
        rightEle.set(-.05);

    }

    public void wearescrewed(){} // method to set the elevator to one motor mode and the other becomes follower
    

    public void wearescrewedbutstilltryandwork(){

        leftEle.follow(rightEle);


        double goalHeight = 0;

        leftEle.set(rightPID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
        rightEle.set(rightPID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
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

    public void periodic(){
        SmartDashboard.putNumber("rightmoroPos", rightEle.getEncoder().getPosition());
        SmartDashboard.putNumber("leftMotorPos", leftEle.getEncoder().getPosition());
        SmartDashboard.putNumber("elbow", elbow.getEncoder().getPosition());

    }

    
    
}
