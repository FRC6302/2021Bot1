/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class DriveGTA extends CommandBase {
  private final DriveTrain driveTrain;
  //AHRS gyro;
  //boolean gyroIsCalibrating;

  /**
   * Creates a new DriveGTA.
   */
  public DriveGTA(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //change to 1 for light mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double scaledStickInput = Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickX) * Constants.turningRate;
    
    double triggerVal = Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger) 
    - Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger);
    
    //squaring the trigger values make them less sensitive when you barely press down on them. 
    //double rightTriggerSquared = Math.pow(Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger), 2);
    //double leftTriggerSquared = Math.pow(Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger), 2);
    //double triggerVal = rightTriggerSquared - leftTriggerSquared;

    driveTrain.setLeftMotors(triggerVal + scaledStickInput);
    driveTrain.setRightMotors(triggerVal - scaledStickInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
