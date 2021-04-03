/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class MoveStraight extends CommandBase {
  DriveTrain driveTrain;
  double leftCommand, rightCommand, speed, moveTime;
  Timer timer;
  double initialYaw, currentYaw;
  double motorInput;
  private boolean finished = false;
  
  /**
   * Creates a new MoveStraight.
   */
  public MoveStraight(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    timer = new Timer();
    moveTime = Constants.MoveTime;  
    speed = Constants.leftMotorsMoveSpeed;
  }  
  
   //runs when all 4 paramaters are inputted in to the command call
  public MoveStraight(DriveTrain driveTrain, double speed, double moveTime) {
    this.driveTrain = driveTrain;
    this.speed = speed;
    addRequirements(driveTrain);
    timer = new Timer();   
    this.moveTime = moveTime; 
  }  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //it moves at the same angle that it started at, so the initial has to be recorded for comparison to current
      initialYaw = NavX.getGyroYaw();
    timer.reset();
    timer.start();
    while(timer.get() < moveTime)
    {
      currentYaw = NavX.getGyroYaw();
      leftCommand = speed + ((initialYaw - currentYaw) / 40);
      rightCommand = speed - ((initialYaw - currentYaw) / 40);
      driveTrain.setLeftMotors(leftCommand);
      driveTrain.setRightMotors(rightCommand);
    }
    finished = true; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
