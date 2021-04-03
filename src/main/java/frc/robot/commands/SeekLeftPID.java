/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SeekLeftPID extends PIDCommand {
  static final double kp = 0.02; //0.02
  static final double ki = 0.001; //0.001
  static final double kd = 0.003; //0.003
  final double positionTolerance = 1; //in degrees
  final double velocityTolerance = 1; //ft per second maybe

  /**
   * Creates a new SeekLeftPID.
   */
  public SeekLeftPID(DriveTrain driveTrain) {
    super( //TO DO: find a way to make this work when there no value for tx. Or implement a PID controller into SeekLeft and delete this
        // The controller that the command will use
        new PIDController(kp, ki, kd),
        // This should return the measurement
        Limelight::getX,
        // This should return the setpoint (can also be a constant)
        0.0,
        // This uses the output
        (output) -> {
          SmartDashboard.putNumber("seekLeft pid output", output);
          driveTrain.setLeftMotors(-output);
          driveTrain.setRightMotors(output);
        },
        driveTrain);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(positionTolerance, velocityTolerance);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
