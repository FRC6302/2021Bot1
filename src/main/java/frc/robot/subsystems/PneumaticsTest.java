// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsTest extends SubsystemBase {
  Compressor compressor = new Compressor(0);
  boolean enabled;

  Solenoid exampleSolenoid = new Solenoid(1);
  DoubleSolenoid exampleDouble = new DoubleSolenoid(1, 2);

  /** Creates a new PneumaticsTest. */
  public PneumaticsTest() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    enabled = compressor.enabled();
  }
}
