// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;


public class DistanceSensor extends SubsystemBase {
  private static Rev2mDistanceSensor distanceSensor;

  static double distance;
  static boolean isRangeValid;

  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    distanceSensor.setAutomaticMode(true);
    distanceSensor.setDistanceUnits(Unit.kInches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = distanceSensor.getRange();

    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("Timestamp", distanceSensor.getTimestamp());
    SmartDashboard.putBoolean("is range valid", distanceSensor.isRangeValid());
  }

  public static double getDistance() {
    //distance = distanceSensor.getRange();
    return distance;
  }

  public static boolean getIsRangeValid() {
    return isRangeValid;
  }
}
