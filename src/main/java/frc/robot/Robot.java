/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.NavX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer robotContainer;

  public String trajectoryJSON1;
  public Path path1;
  public static Trajectory trajectory1; 

  public String trajectoryJSON2;
  public Path path2;
  public static Trajectory trajectory2; 


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /*
      Pathweaver is a program built into VS Code that shows an image of the course i need to run and
      lets me draw a trajectory onto the course. I then put this trajectory into my ramsete command.

      I am loading these trajectories right when the robot inits and not in the getRamseteCommand() method
      because they can take a while to load up, and if i did it there, then there would be a delay when
      I called the method or started autonomous mode.
    */

    //change this to change what path is run by the robot
    //the computer looks for the file in 2021Bot1/src/main/deploy, so all these files are in that folder
    //trajectoryJSON1 = "SlalomPW2/PathWeaver/output/Slalom1.wpilib.json";
    //trajectoryJSON1 = "BarrelPW2/PathWeaver/output/Barrel1.wpilib.json";
    trajectoryJSON1 = "BouncePW2/PathWeaver/output/Bounce1.wpilib.json";
    //trajectoryJSON1 = "BouncePW2/PathWeaver/output/Forwards.wpilib.json";
    //trajectoryJSON1 = "GalacticAPW2/PathWeaver/output/GalacticA1.wpilib.json";
    //trajectoryJSON1 = "GalacticBPW2/PathWeaver/output/GalacticB1.wpilib.json";

    //random numbers. If everything runs right, then this trajectory should never be followed.
    Trajectory.State state1 = new Trajectory.State(1., 1., 1., new Pose2d(0, 0, new Rotation2d(0)), 1.);
    
    //The trajectory has to be initialized as something in case the trajectory from the file isnt received.
    //The trajectory is static so i can access it in RobotContainer without making a Robot object
    Robot.trajectory1 = new Trajectory(List.of(state1));

    try {
      path1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(path1);
    } catch (IOException ex) {
      DriverStation.reportError("Trajectory file path probably doesn't exist:", ex.getStackTrace());
    }
    

    //Making a second trajectory in case something has to be done inbetween the robot moving.
    //For example, i might wanna follow a path to a shooting position, turn towards the target,
    //shoot, and then follow another path somewhere else. This couldnt be achieved with just one path.
    trajectoryJSON2 = "BouncePW2/PathWeaver/output/Reverse.wpilib.json";

    //random numbers
    Trajectory.State state2 = new Trajectory.State(1., 1., 1., new Pose2d(0, 0, new Rotation2d(0)), 1.);

    //the trajectory has to be initialized as something in case the trajectory from the file isnt received
    Robot.trajectory2 = new Trajectory(List.of(state2));

    try {
      path2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(path2);
    } catch (IOException ex) {
      DriverStation.reportError("Trajectory file path probably doesn't exist:", ex.getStackTrace());
    }

    //this makes sure the yaw is zero when the robot starts up
    NavX.zeroGyroYaw();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //use when doing a trajectory facing backwards
    //robotContainer.reverseDriveTrain();
    
    NavX.zeroGyroYaw();

    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
