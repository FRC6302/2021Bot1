/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class MoveDistance extends CommandBase {
  DriveTrain driveTrain;
  double targetDistance;
  double speed;

  boolean finished = false;
  double distanceTravelled;
  double motorCommand;

  /**
   * Creates a new MoveDistance.
   */
  public MoveDistance(DriveTrain driveTrain, double targetDistance, double speed) {
    /*pseudocode:
    get average of distance travelled
    left command = speed * curDistance / endDistance ?
    */
    this.driveTrain = driveTrain;
    this.targetDistance = targetDistance;
    this.speed = speed;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //the robot should be moving in a straight line, so the distance should be almost equal.
    //Getting their average maybe accounts for any error. Who knows :)
    distanceTravelled = (driveTrain.getLeftEncDistance() + driveTrain.getRightEncDistance()) / 2;

    motorCommand = speed * (targetDistance - distanceTravelled) / targetDistance;

    driveTrain.setLeftMotors(motorCommand);
    driveTrain.setRightMotors(motorCommand);
    SmartDashboard.putNumber("motor command", motorCommand);

    finished = (distanceTravelled > targetDistance * 0.95);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("finished", finished);
    return finished;
  }
}
