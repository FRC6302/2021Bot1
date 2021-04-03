/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ZeroYawAndTurnRight extends SequentialCommandGroup {
  /**
   * Creates a new ZeroYawAndTurnRight.
   */
  public ZeroYawAndTurnRight(DriveTrain driveTrain) {
    //super(new ZeroYaw(), new TurnRight(driveTrain)); 
    super(new InstantCommand(NavX::zeroGyroYaw), new TurnRight(driveTrain));
    //zeroes yaw then turns right
    /* 
    Using a method reference here so i dont need an entire ZeroYaw subsystem to just do one thing. 
    The super() call doesnt allow method reference arguments so you have to make it inside an instant command
    */
  }
   public ZeroYawAndTurnRight(DriveTrain driveTrain, double targetAngle) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new InstantCommand(NavX::zeroGyroYaw), new TurnRight(driveTrain, targetAngle));
  }
}