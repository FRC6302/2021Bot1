/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class GetInRange extends CommandBase {
  private final DriveTrain driveTrain;
  private boolean finished = false;
  double motorAdjust;
  double area;
  double targetFound;

  /**
   * Creates a new GetInRange.
   */
  public GetInRange(DriveTrain driveTrain) {
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
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //0 is dark
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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry ta = table.getEntry("ta"); //ranges from 0 to 100% of image
    NetworkTableEntry tv = table.getEntry("tv");

    double area = ta.getDouble(0.0);
    double targetFound = tv.getDouble(0.0);

    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightTargetFound", targetFound);

    /*
    if (targetFound == 1)
    {
      motorAdjust = (Constants.limelightTargetArea - area) * 0.6; //scaling downs makes robot less likely to go past target distance
    }
    else
    {
      motorAdjust = -Constants.limelightGetInRangeSpeed;
    }
    */
    motorAdjust = (targetFound == 1)? (Constants.limelightTargetArea - area) * 0.6 : -Constants.limelightGetInRangeSpeed;
    /*
    if (targetFound == 1) 
    {
      if (area < Constants.limelightTargetArea)
      {
        motorAdjust = Constants.limelightGetInRangeSpeed;
      }
      else
      {
        motorAdjust = -Constants.limelightGetInRangeSpeed;
      }
      double deadzone = 0.05;
      finished = (area >= Constants.limelightTargetArea - deadzone && area <= Constants.limelightTargetArea + deadzone);
    }
    else
    {
      motorAdjust = Constants.limelightGetInRangeSpeed;
    }
    */
    
    driveTrain.setLeftMotors(motorAdjust);
    driveTrain.setRightMotors(motorAdjust);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //changes back to normal cam mode
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    //driveTrain.stopDrive();
    area = 0;
    targetFound = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
