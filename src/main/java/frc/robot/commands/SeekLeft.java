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
import frc.robot.subsystems.Limelight;


public class SeekLeft extends CommandBase {
  private final DriveTrain driveTrain;
  private boolean finished = false;

  /**
   * Creates a new SeekLeft.
   */
  public SeekLeft(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    //this is a ternary operator. Google it
    double steeringAdjust = (Limelight.getTargetFound() == 1) ? -Limelight.getX() / 30 : Constants.limelightSeekSpeed;
    
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
    finished = Math.abs(steeringAdjust) < 3;
    
    driveTrain.setLeftMotors(-steeringAdjust);
    driveTrain.setRightMotors(steeringAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //changes back to normal cam mode
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    //driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
