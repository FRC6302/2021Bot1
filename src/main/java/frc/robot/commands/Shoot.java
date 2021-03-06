/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  Shooter shooter;

  /**
   * Creates a new Shoot.
   */
  public Shoot(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterMotors(Constants.shootSpeed);
    //shooter.setShooterMotorsWith2Encoders(Constants.shootSpeed);
    
    /*double leftLEncRate = shooter.getLeftLEncoderRate();
    double leftREncRate = shooter.getLeftREncoderRate();
    double rightLEncRate = shooter.getRightLEncoderRate();
    double rightREncRate = shooter.getRightREncoderRate();

    SmartDashboard.putNumber("LeftLEncRate", leftLEncRate);
    SmartDashboard.putNumber("LeftREncRate", leftREncRate);
    SmartDashboard.putNumber("RightLEncRate", rightLEncRate);
    SmartDashboard.putNumber("RightREncRate", rightREncRate);*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
