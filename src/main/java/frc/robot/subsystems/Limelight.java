package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");  

  public double getOffsetYaw() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getOffsetPitch() { 
    return table.getEntry("ty").getDouble(0.0);
  }

  public Pose2d getPose(){
    return null;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Limelight X dist", getOffsetYaw());
    SmartDashboard.putNumber("Limelight Y dist", getOffsetPitch());
  }

}