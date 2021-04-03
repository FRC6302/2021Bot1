/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class TurnToYawZero extends CommandBase {
  DriveTrain driveTrain;
  double gyroYaw = NavX.getGyroYaw();
  double curGyroYaw = NavX.getGyroYaw();
  double oldGyroYaw = NavX.getGyroYaw();
  boolean firstPartDone;
  double DEADZONE = 30;
  double motorInput;

  //private boolean finished = false;

  /**
   * Creates a new TurnToYawZero.
   */
  public TurnToYawZero(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroYaw = NavX.getGyroYaw();
    //motorInput = Math.pow(gyroYaw, 3) * 0.0000004;
    if (gyroYaw > DEADZONE || gyroYaw < -DEADZONE) { motorInput = gyroYaw / 300; }
    else { motorInput = gyroYaw / 50; }
    driveTrain.setLeftMotors(-motorInput);
    driveTrain.setRightMotors(motorInput); 
    /*oldGyroYaw = NavX.getGyroYaw();
    if (oldGyroYaw > DEADZONE || oldGyroYaw < -DEADZONE) { motorInput = oldGyroYaw / 300; }
    else { motorInput = oldGyroYaw / 50; }
    driveTrain.setLeftMotors(-motorInput);
    driveTrain.setRightMotors(motorInput);
    curGyroYaw = NavX.getGyroYaw();
    motorInput = oldGyroYaw - curGyroYaw;
    driveTrain.setLeftMotors(-motorInput);
    driveTrain.setRightMotors(motorInput);*/
    /*if (gyroYaw > DEADZONE || gyroYaw < -DEADZONE)
    {
      initialize();
    }
    /*
    if (gyroYaw > DEADZONE || gyroYaw < -DEADZONE) 
    {
      driveTrain.setLeftMotors(Constants.turnToZeroYawSpeed);
      driveTrain.setRightMotors(-Constants.turnToZeroYawSpeed);
    }
    else 
    { */
      //driveTrain.setLeftMotors(-Constants.turnToZeroYawSpeed * 0.5);
      //driveTrain.setRightMotors(Constants.turnToZeroYawSpeed * 0.5);
    //} */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return gyroYaw < 2 && gyroYaw > -2; //in degrees
    //return firstPartDone && ;
    return false;
  }
}
