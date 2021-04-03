/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatteryVoltage extends SubsystemBase {
  static double initBatteryVoltage = RobotController.getBatteryVoltage();
  static double curBatteryVoltage;

  /**
   * Creates a new BatteryVoltage.
   */
  public BatteryVoltage() { //class is never instantiated so need for constructor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    curBatteryVoltage = RobotController.getBatteryVoltage();
    SmartDashboard.putNumber("current battery voltage", curBatteryVoltage);
    SmartDashboard.putNumber("initial battery voltage", initBatteryVoltage);
  }

  public static double getCurBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  public static double getInitBatteryVoltage() {
    return initBatteryVoltage;
  }
}
