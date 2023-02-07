package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final DoubleSolenoid inSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 11);
    
    public Intake() {
        Down();        
    }

    public void Down() {
        inSolenoid.set(Value.kReverse);
    }

    public void Up() {
        inSolenoid.set(Value.kForward);
    }

    public boolean getIntake() {
        return inSolenoid.get().equals(DoubleSolenoid.Value.kForward);
    }

}
