/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecDriveTrain extends SubsystemBase {
  /*
  using WPI_TalonsSRX class instead of TalonSRX because the MecanumDrive() constructor needed to
  implement mecanum does not accept the TalonSRX class but accepts this one because the WPI class
  inherits the SpeedController class, which is a valid parameter for the constructer
  */
  public WPI_TalonSRX motorL1;
  public WPI_TalonSRX motorL2;
  public WPI_TalonSRX motorR1;
  public WPI_TalonSRX motorR2;

  /**
   * Creates a new MecDriveTrain.
   */
  public MecDriveTrain() {
    motorL1 = new WPI_TalonSRX(Constants.mecMotorL1Value);
    motorL2 = new WPI_TalonSRX(Constants.mecMotorL2Value);
    motorR1 = new WPI_TalonSRX(Constants.mecMotorR1Value);
    motorR2 = new WPI_TalonSRX(Constants.mecMotorR2Value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopDrive(){
    motorL1.set(ControlMode.PercentOutput, 0);
    motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    motorR2.set(ControlMode.PercentOutput, 0);
  }
}
