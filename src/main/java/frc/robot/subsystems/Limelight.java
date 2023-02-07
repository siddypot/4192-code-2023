package frc.robot.subsystems;
import javax.management.relation.Relation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {


  boolean blueAlliance; //1 is blue 0 is red

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");  


  public Limelight(boolean alliance){
    blueAlliance = alliance;
    
  }

  public double getOffsetYaw() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getOffsetPitch() { 
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getHorizontalLength(){
    return table.getEntry("thor").getDouble(0.0);
  }

  public double getVerticalLength(){
    return table.getEntry("tvert").getDouble(0.0);
  }


  public Pose3d getPose(){

    if(blueAlliance){

      double[] pose =  table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
      return new Pose3d(new Translation3d(pose[0],pose[1],pose[2]), new Rotation3d(pose[3],pose[4],pose[5]));

    }
    else {
      double[] pose =  table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
      return new Pose3d(new Translation3d(pose[0],pose[1],pose[2]), new Rotation3d(pose[3],pose[4],pose[5]));
    }
  }

  public Pose2d getPose2d(){
    
    return getPose().toPose2d();
  }

  public void switchMode(boolean reflective){ 

    //PUT RETRO REFLECTIVE ON PIPELINE 0
    //APRIL TAG ON PIPELINE 1
    
    table.getEntry("pipeline").setNumber( (reflective) ? 0 : 1);
  }

  @Override
  public void periodic() {
    
  }

}