/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class TurnRight extends PIDCommand {
  static final double kp = 0.02; //0.02
  static final double ki = 0.001; //0.001
  static final double kd = 0.003; //0.003
  final double positionTolerance = 1; //in degrees
  final double velocityTolerance = 1; //ft per second maybe
  
  /**
   * Creates a new TurnRight902.
   */
  public TurnRight(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(new PIDController(kp, ki, kd), NavX::getGyroYaw, 90.0, driveTrain::usePIDOutput, driveTrain);  
    // Set the controller to be continuous (because it is an angle controller)
     getController().enableContinuousInput(-180, 180);
     // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
     // setpoint before it is considered as having reached the reference
     getController().setTolerance(positionTolerance, velocityTolerance);
     SmartDashboard.putNumber("testNumber", NavX.getGyroYaw());
  }

  public TurnRight(DriveTrain driveTrain, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(new PIDController(kp, ki, kd), NavX::getGyroYaw, targetAngle, driveTrain::usePIDOutput, driveTrain); 
    // Set the controller to be continuous (because it is an angle controller)
     getController().enableContinuousInput(-180, 180);
     // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
     // setpoint before it is considered as having reached the reference
     getController().setTolerance(positionTolerance, velocityTolerance);
     SmartDashboard.putNumber("testNumber", NavX.getGyroYaw());
  }
  /*
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("number", 5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
  */

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return getController().atSetpoint();
    return false;
  }
}
