/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BatteryVoltage;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonBouncePath extends SequentialCommandGroup {
  //this makes sure that the same output will be achieved no matter of battery voltage
  static double voltageK = 12.8 / BatteryVoltage.getInitBatteryVoltage(); //12.8 is max voltage I've seen

  // SendableChooser<Double> firstLeft = new SendableChooser<>();
  // SendableChooser<Double> firstRight = new SendableChooser<>();
  // SendableChooser<Double> firstTime = new SendableChooser<>();
  /**
   * Creates a new AutonBouncePath.
   */
  public AutonBouncePath(DriveTrain driveTrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // starts: centered with a blue line bisecting robot,front of the start
    // zone,parallel to front blue line
    super(new Move(driveTrain, 0, 0, 0.1), // makes the starts more consistent
    new Move(driveTrain, 0.22, 0.4, 1.85 * voltageK), //turns towards the first cone
    new Move(driveTrain, 0.3, 0.3, 0.24 * voltageK), //hits first cone
    new Move(driveTrain, 0, 0, 0.14 * voltageK), //makes robot slip less
    new Move(driveTrain, -0.4, -0.27, 2.35 * voltageK) //backs up towards next turn
    //new Move(driveTrain, -0.4, -0.13, 1.85 * voltageK), //turns around cone
    //new Move(driveTrain, -0.4, -0.26, 0.12 * voltageK), //makes robot jerk less
    //new Move(driveTrain, -0.4, -0.4, 1.10 * voltageK), //hits second cone
    //new Move(driveTrain, 0, 0, 0.3 * voltageK), //makes robot slip less
    //new Move(driveTrain, 0.32, 0.4, 1.22 * voltageK) //0.75 0.8 moves towards next turn
    //new Move(driveTrain, 0.25, 0.8, 0.95 * voltageK), //turns around second cone
    //new Move(driveTrain, 0.5, 0.8, 0.1 * voltageK), //makes robot jerk less
    //new MoveStraight(driveTrain, 0.8, 0.3 * voltageK) //hits second cone */
    );
  }
}
