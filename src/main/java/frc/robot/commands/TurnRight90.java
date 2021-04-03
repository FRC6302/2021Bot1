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

public class TurnRight90 extends CommandBase {
  DriveTrain driveTrain;
  double gyroYaw;
  double TARGET_VALUE = 90;
  double motorInput;
  
  /**
   * Creates a new TurnRight90.
   */
  public TurnRight90(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NavX.zeroGyroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroYaw = NavX.getGyroYaw();
    motorInput = (TARGET_VALUE - gyroYaw) / 200; //dont change this divisor without testing the new value
    driveTrain.setLeftMotors(motorInput);
    driveTrain.setRightMotors(-motorInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
