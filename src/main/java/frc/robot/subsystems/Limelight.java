/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  static double x = 0, y = 0, targetFound = 0;

  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    try {
      //camera controls
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //1 is light mode
      //changing camMode can be used to switch between the normal cam and the darkened targeting mode
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
      }
      catch (RuntimeException ex){
        DriverStation.reportError("error setting limelight values because: " + ex.getMessage(), true);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    NetworkTableEntry tx = table.getEntry("tx"); 
    NetworkTableEntry ty = table.getEntry("ty"); 
    //NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    //read values periodically
    Limelight.x = tx.getDouble(0.0); //ranges from -29.8 to 29.8 degrees for LL2
    Limelight.y = ty.getDouble(0.0); //ranges from -24.85 to 24.85 degrees for LL2
    //double area = ta.getDouble(0.0); //ranges from 0 to 100% of image
    Limelight.targetFound = tv.getDouble(0.0);

    //posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area); 
    SmartDashboard.putNumber("LimelightTargetFound", targetFound);
  }

  public static double getX() {
    return x;
  }

  public static double getY() {
    return y;
  }

  public static double getTargetFound() {
    return targetFound;
  }
}
