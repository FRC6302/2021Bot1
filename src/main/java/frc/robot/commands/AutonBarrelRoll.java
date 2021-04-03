/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonBarrelRoll extends SequentialCommandGroup {
  /**
   * Creates a new AutonBarrelRoll.
   */
  public AutonBarrelRoll(DriveTrain driveTrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    /* super(new Move(driveTrain, 0.6, 0.6, 1.5), //almost straight up to the first turn 
      new Move(driveTrain, 0.5, 0.2, 4.15),
      new Move(driveTrain, 0.6, 0.6, 0.5)); */

      /*super(new MoveStraight(driveTrain, 0.8, 1.15), //straight up to the first turn 
      new Move(driveTrain, 0.6, 0.23, 3.2), //turns around first cone
      new Move(driveTrain, 0.7, 0.5, 0.1), //acts as a buffer so the robot doesnt jerk as much
      new MoveStraight(driveTrain, 0.8, 0.3)); //moves to second cone*/
      super(new MoveDistance(driveTrain, 2, 0.8));
  }
}
