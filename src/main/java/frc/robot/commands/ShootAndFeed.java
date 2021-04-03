/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndFeed extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndFeed.
   */
  public ShootAndFeed(Shooter shooter, Shooter feeder) {
    // Add your commands in the super() call, e.g.
    super(new TimedShoot(shooter), new ParallelShootAndFeed(shooter, feeder));
    //shoots for a certain amount of time(to speed up the wheels) and then shoots and feeds at the same time
  }
}
