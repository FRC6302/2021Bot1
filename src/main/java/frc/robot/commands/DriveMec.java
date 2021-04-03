/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.MecDriveTrain;

public class DriveMec extends CommandBase {
  /**
   * Creates a new DriveMec.
   */
  MecDriveTrain mecDriveTrain;
  double ySpeed;
  double xSpeed;
  double zRotation;
  
  MecanumDrive mecanumDrive;

  public DriveMec(MecDriveTrain mecDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.mecDriveTrain = mecDriveTrain;
  addRequirements(mecDriveTrain);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //0 is dark
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ySpeed = Robot.robotContainer.getDriverRawAxis(Constants.leftStickY);
    xSpeed = Robot.robotContainer.getDriverRawAxis(Constants.leftStickX);
    zRotation = Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger) 
    - Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger);

    mecanumDrive = new MecanumDrive(mecDriveTrain.motorL1, 
    mecDriveTrain.motorL2, mecDriveTrain.motorR1, mecDriveTrain.motorR2);
    //might have to switch the positions of xSpeed and ySpeed
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mecDriveTrain.stopDrive();   
    //idk why i have to close it but i do
    //mecanumDrive.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
