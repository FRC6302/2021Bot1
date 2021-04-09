// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveDistancePID extends PIDCommand {
  //pid controller constants for fully charged battery
  private static final double kp = 0.6, ki = 0, kd = 0;

  //pid controller tolerances
  private static final double positionTolerance = 0.08; //in meters
  private static final double velocityTolerance = 0.05; //idk the units for this. Its not m/s

  /** Creates a new MoveDistancePID. */
  public MoveDistancePID(DriveTrain driveTrain, double distanceMeters) {
    super(
        // The controller that the command will use
        new PIDController(kp, ki, kd),
        // This should return the measurement
        driveTrain::getAverageEncDistance,
        // This should return the setpoint (can also be a constant)
        distanceMeters,
        // This uses the output
        (output) -> {
          // Use the output here
          driveTrain.setLeftMotors(output);
          driveTrain.setRightMotors(output);
        },
        //requires the drivetrain
        driveTrain
        );

    // Use addRequirements() here to declare subsystem dependencies.

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(positionTolerance, velocityTolerance); //in meters and meters per idk
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("velocity error", getController().getVelocityError());

    //ends when the target distance is less than 0.01 m away and robot is moving less than 0.01 m/s
    return getController().atSetpoint();
  }
}
