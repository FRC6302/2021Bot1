// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimitSwitch extends SubsystemBase {
  DigitalOutput magLimSwitch = new DigitalOutput(Constants.magLimSwitchPort);
  boolean inRange = false;

  /** Creates a new LimitSwitch. */
  public LimitSwitch() {
  }

  //TODO: write code to test out limit switch on a motor somehow? Nothing on there now moves around
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inRange = magLimSwitch.get();
  }

  public boolean getInRange() {
    return inRange;
  }
}
