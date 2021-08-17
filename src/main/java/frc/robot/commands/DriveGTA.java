/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class DriveGTA extends CommandBase {
  private final DriveTrain driveTrain;
  private double scaledStickInput;
  private double triggerVal;
  //private double rightTriggerSquared, leftTriggerSquared;
  //AHRS gyro;
  //boolean gyroIsCalibrating;

  private double currentAccelX = 0;
  private double lastAccelX = 0;
  private double jerkX = 0;

  private double currentAccelY = 0;
  private double lastAccelY = 0;
  private double jerkY = 0;

  boolean collisionDetected = false;

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
    driveTrain.unReverseDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scaledStickInput = Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickX) * Constants.turningRate;
    
    triggerVal = Constants.maxDriveSpeed * (Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger) 
      - Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger));
    
    //squaring the trigger values make them less sensitive when you barely press down on them. 
    //rightTriggerSquared = Math.pow(Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger), 2);
    //leftTriggerSquared = Math.pow(Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger), 2);
    //triggerVal = Constants.maxDriveSpeed * (rightTriggerSquared - leftTriggerSquared);

    driveTrain.setLeftMotors(triggerVal + scaledStickInput);
    driveTrain.setRightMotors(triggerVal - scaledStickInput);

    currentAccelX = NavX.getGyroAccelX();
    jerkX = currentAccelX - lastAccelX;
    lastAccelX = NavX.getGyroAccelX();

    currentAccelY = NavX.getGyroAccelY();
    jerkY = currentAccelY - lastAccelY;
    lastAccelY = NavX.getGyroAccelY();

    if (Math.abs(jerkX) > Constants.jerkThreshold || Math.abs(jerkY) > Constants.jerkThreshold)
    {
      collisionDetected = true;
    }

    SmartDashboard.putNumber("jerkX", jerkX);
    SmartDashboard.putNumber("jerkY", jerkY);
    SmartDashboard.putBoolean("collision detected?", collisionDetected);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.unReverseDrive();
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
