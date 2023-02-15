package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

public class Elevator {

    private final CANSparkMax leftEle;
    private final CANSparkMax rightEle;

    private final CANSparkMax elbow;

    PIDController leftPID = new PIDController(1, 0, 0);
    PIDController rightPID = new PIDController(1, 0, 0);

    PIDController elbowPID = new PIDController(1, 0, 0);


    public Elevator(){

        elbow = new CANSparkMax(0, MotorType.kBrushless);

        leftEle = new CANSparkMax(0, MotorType.kBrushless);
        rightEle = new CANSparkMax(0, MotorType.kBrushless);

        leftEle.setInverted(true);
        rightEle.setInverted(false);


        motorSetup(rightEle);
        motorSetup(leftEle);

        motorSetup(elbow);
        
    }


    public void motorSetup(CANSparkMax c){
        c.setSmartCurrentLimit(20);
        c.restoreFactoryDefaults();
        

    }
    private void raiseEle(double goalHeight){

        rightEle.set(rightPID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
        leftEle.set(leftPID.calculate(leftEle.getEncoder().getPosition(), goalHeight));

    }

    public void wearescrewed(){} // method to set the elevator to one motor mode and the other becomes follower
    

    public void wearescrewedbutstilltryandwork(){

        leftEle.follow(rightEle);


        double goalHeight = 0;

        leftEle.set(rightPID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
        rightEle.set(rightPID.calculate(rightEle.getEncoder().getPosition(), goalHeight));
    }

    public void gripperOut(boolean direction){

        double out = 0; //change
        

        elbow.set(
            (direction) ? elbowPID.calculate(elbow.getEncoder().getPosition(), out )
                        : elbowPID.calculate(elbow.getEncoder().getPosition(), out));

    }

    public void gripperIn(boolean direction){

        double in = 0; //change
        

        elbow.set(
            (direction) ? elbowPID.calculate(elbow.getEncoder().getPosition(), in)
                        : elbowPID.calculate(elbow.getEncoder().getPosition(), in));

    }

    
    
}
