// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;

public class MoveDistanceSensor extends CommandBase {
  DriveTrain driveTrain;
  double targetDistance;
  double speed;

  boolean finished = false;
  double motorCommand;
  double distance;

  /**
   * Creates a new MoveDistance.
   */
  public MoveDistanceSensor(DriveTrain driveTrain, double targetDistance, double speed) {
    /*pseudocode:
    get average of distance travelled
    left command = speed * curDistance / endDistance ?
    */
    this.driveTrain = driveTrain;
    this.targetDistance = targetDistance;
    this.speed = speed;

    addRequirements(driveTrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //so that you can run the command more than once. When you press the button again, finished resets to false
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = DistanceSensor.getDistance();

    motorCommand = speed;
    

    driveTrain.setLeftMotors(motorCommand);
    driveTrain.setRightMotors(motorCommand);
    SmartDashboard.putNumber("motor command", motorCommand);

    /*while (distance > targetDistance) {
      distance = DistanceSensor.getDistance();
      finished = false;

      //MotorSafety.feed();

      //updates motor values because they would not be updated otherwise. prevents error 
      driveTrain.periodic();

      //DistanceSensor.periodic();
      
    }
    */

    if (distance < targetDistance) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
    //finished = false;
  }

  // Returns true when the command should end.
  @Override 
  public boolean isFinished() {
    return finished;
  }
}
