// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SenseDistance extends CommandBase {
  DriveTrain driveTrain;

  private Rev2mDistanceSensor distanceSensor;

  /** Creates a new SenseDistance. */
  public SenseDistance(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrain = driveTrain;

    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceSensor.setAutomaticMode(true);
    distanceSensor.setDistanceUnits(Unit.kInches);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("range", distanceSensor.getRange());
    //SmartDashboard.putNumber("Timestamp", distanceSensor.getTimestamp());
    //SmartDashboard.putBoolean("is range valid", distanceSensor.isRangeValid());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //distanceSensor.setAutomaticMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
