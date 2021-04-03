/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SeekRight extends CommandBase {
  private final DriveTrain driveTrain;
  private boolean finished = false;

  /**
   * Creates a new SeekRight.
   */
  public SeekRight(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
    //camera controls
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    //changing camMode can be used to switch between the normal cam and the darkened targeting mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    }
    catch (RuntimeException ex){
      DriverStation.reportError("error setting limelight values because: " + ex.getMessage(), true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    //idk how these are different than the lines below but they might be needed

    
    //try {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    NetworkTableEntry tx = table.getEntry("tx"); 
    NetworkTableEntry ty = table.getEntry("ty"); 
    //NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);
    double targetFound = tv.getDouble(0.0);
    
    
    /*
    //gets tx as a double. 0.0 is returned if no value is found
    double x = table.getEntry("tx").getDouble(0.1); //ranges from -29.8 to 29.8 degrees for LL2
    double y = table.getEntry("ty").getDouble(0.1); //ranges from -24.85 to 24.85 degrees for LL2
    //double area = table.getEntry("ta").getDouble(0.0); //ranges from 0 to 100% of image
    //targetFound might have to be changed to a double
    boolean targetFound = table.getEntry("tv").getBoolean(false);
    */
    
    //posts to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightTargetFound", targetFound);

    //this is a ternary operator. Google it
    double steeringAdjust = (targetFound == 1) ? x / 100 : Constants.limelightSeekSpeed;
    
    /*
    //this does the same thing as the ternary operator above
    double steeringAdjust;
    if (targetFound == 0) //0 means target is not found
    {
      steeringAdjust = Constants.limelightSeekSpeed;
    }
    else //this runs when the target is in view of camera
    {
      //driveTrain.stopDrive();
      //finished = true;
      steeringAdjust = x / 100;
    }
    */
    
    //3 degrees to the right or left of target and the program ends
    //finished = Math.abs(steeringAdjust) < 3;
    
    driveTrain.setLeftMotors(steeringAdjust);
    driveTrain.setRightMotors(-steeringAdjust);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //changes back to normal cam mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    //driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
