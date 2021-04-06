/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonBarrelRoll;
import frc.robot.commands.AutonBouncePath;
import frc.robot.commands.DriveGTA;
import frc.robot.commands.Feed;
import frc.robot.commands.GetInRange;
import frc.robot.commands.Move;
import frc.robot.commands.MoveStraight;
import frc.robot.commands.ParallelShootAndFeed;
import frc.robot.commands.SeekLeft;
import frc.robot.commands.SeekLeftPID;
import frc.robot.commands.SeekRight;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAndFeed;
import frc.robot.commands.SuckBalls;
import frc.robot.commands.TurnRight90;
import frc.robot.commands.TurnRight;
import frc.robot.commands.TurnToYawZero;
import frc.robot.commands.ZeroYawAndTurnRight;
import frc.robot.subsystems.BatteryVoltage;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public static XboxController driverController;
  //public static XboxController operatorController;

  private final DriveTrain driveTrain;
  private final DriveGTA driveGTA;
  //private final MecDriveTrain mecDriveTrain;
  //private final DriveMec driveMec;

  private final Intake intake;
  private final SuckBalls suckBalls;

  private final Shooter shooter;
  private final Shooter feeder;
  private final Shoot shoot;
  private final Feed feed;
  private final ShootAndFeed shootAndFeed;
  private final ParallelShootAndFeed parallelShootAndFeed;

  private final Limelight limelight;
  private final SeekLeft seekLeft;
  private final SeekLeftPID seekLeftPID;
  private final SeekRight seekRight;
  private final GetInRange getInRange;

  private final Move move;

  private final NavX navX;
  private final TurnToYawZero turnToYawZero;
  private final TurnRight90 turnRight90;
  private final TurnRight turnRight;
  private final ZeroYawAndTurnRight zeroYawAndTurnRight;
  private final MoveStraight moveStraight;

  private final BatteryVoltage batteryVoltage;
  
  AutonBarrelRoll autonBarrelRoll;
  AutonBouncePath autonBouncePath;

  SendableChooser<Command> chooser = new SendableChooser<>();
  //Smart Dashboard cannot be set to "Editable" if you want to select an option for auton

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    driverController = new XboxController(Constants.driverControllerPort);
    //operatorController = new XboxController(Constants.operatorControllerPort);
    
    driveTrain = new DriveTrain();
    driveGTA = new DriveGTA(driveTrain);
    driveGTA.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveGTA);

    //mecDriveTrain = new MecDriveTrain();
    //driveMec = new DriveMec(mecDriveTrain);
    //driveMec.addRequirements(mecDriveTrain);
    //mecDriveTrain.setDefaultCommand(driveMec);

    shooter = new Shooter();
    feeder = new Shooter();
    shoot = new Shoot(shooter);
    shoot.addRequirements(shooter);
    feed = new Feed(shooter);
    feed.addRequirements(shooter);
    shootAndFeed = new ShootAndFeed(shooter, feeder);
    shootAndFeed.addRequirements(shooter, feeder);
    parallelShootAndFeed = new ParallelShootAndFeed(shooter, feeder);
    parallelShootAndFeed.addRequirements(shooter, feeder);

    intake = new Intake();
    suckBalls = new SuckBalls(intake);
    suckBalls.addRequirements(intake);

    limelight = new Limelight();
    seekLeft = new SeekLeft(driveTrain);
    seekLeft.addRequirements(driveTrain, limelight);
    seekLeftPID = new SeekLeftPID(driveTrain);
    seekLeftPID.addRequirements(driveTrain);
    seekRight = new SeekRight(driveTrain);
    seekRight.addRequirements(driveTrain);
    //seekRight.addRequirements(limelight);
    getInRange = new GetInRange(driveTrain);
    getInRange.addRequirements(driveTrain);

    move = new Move(driveTrain);
    move.addRequirements(driveTrain);

    navX = new NavX(); //NavX class must be instantiated or the code will never run and it wont give values
    turnToYawZero = new TurnToYawZero(driveTrain);
    turnToYawZero.addRequirements(driveTrain, navX); 
    //just added this requirement so i dont get an error saying that navX is not used. Is not needed
    turnRight90 = new TurnRight90(driveTrain);
    turnRight90.addRequirements(driveTrain);
    turnRight = new TurnRight(driveTrain);
    turnRight.addRequirements(driveTrain);
    zeroYawAndTurnRight = new ZeroYawAndTurnRight(driveTrain);
    zeroYawAndTurnRight.addRequirements(driveTrain);
    moveStraight = new MoveStraight(driveTrain);
    moveStraight.addRequirements(driveTrain);

    batteryVoltage = new BatteryVoltage();

    autonBarrelRoll = new AutonBarrelRoll(driveTrain);
    autonBarrelRoll.addRequirements(driveTrain);
    autonBouncePath = new AutonBouncePath(driveTrain);
    autonBouncePath.addRequirements(driveTrain, batteryVoltage); //doesnt actually need battery, I just dont want errors

    chooser.addOption("Auton Barrel Roll", autonBarrelRoll);
    chooser.addOption("Auton Bounce Path", autonBouncePath);
    chooser.setDefaultOption("Move (default)", move);
    SmartDashboard.putData("Auton Chooser", chooser);
    
    // Configure the button bindings
    configureButtonBindings();
  }

  public double getDriverRawAxis(final int axis){
    try {
      return driverController.getRawAxis(axis);
    }
    catch(final RuntimeException exception) {
      DriverStation.reportError("Error getting raw axis because: " + exception.getMessage(), true);
    }
    //this error might have something to do with the squared values in DriveGTA
    return 0;
  }

  public double getDriverDeadzoneAxis(final int axis){
    try {
    final double rawValue = driverController.getRawAxis(axis);
    return (Math.abs(rawValue) <= Constants.deadzone) ? 0.0 : rawValue;
    }
    catch(final RuntimeException exception) {
      DriverStation.reportError("Error getting raw axis or returning deadzone axis because: " + exception.getMessage(), true);
    }
    return 0;
  }
  /*
  public double getOperatorDeadzoneAxis(int axis){
    double rawValue = operatorController.getRawAxis(axis);
    return Math.abs(rawValue) < Constants.deadzone ? 0.0 : rawValue;
  }
  */

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton limelightTargetButton = new JoystickButton(driverController, Constants.limelightTargetButton);
    //limelightTargetButton.whileHeld(new SeekLeft(driveTrain));
    limelightTargetButton.whileHeld(new SeekLeftPID(driveTrain));

    //final JoystickButton moveButton = new JoystickButton(driverController, Constants.moveButton);
    //moveButton.whenPressed(new Move(driveTrain, 0.3, -0.3, 1));

    //final JoystickButton limelightGetInRangeButton = new 
      //JoystickButton(driverController, Constants.limelightGetInRangeButton);
    //limelightGetInRangeButton.whileHeld(new GetInRange(driveTrain));

    final JoystickButton zeroYawButton = new JoystickButton(driverController, Constants.zeroYawButton);
    zeroYawButton.whenPressed(NavX::zeroGyroYaw); //this is a method reference

    //final JoystickButton turnToYawZeroButton = new JoystickButton(driverController, Constants.turnToYawZeroButton);
    //turnToYawZeroButton.whileHeld(new TurnToYawZero(driveTrain));

    //final JoystickButton turnRightButton = new JoystickButton(driverController, Constants.turnRightButton);
    //turnRightButton.whileHeld(new ZeroYawAndTurnRight(driveTrain));

    final JoystickButton moveStraightButton = new JoystickButton(driverController, Constants.moveStraightButton);
    moveStraightButton.whenPressed(new MoveStraight(driveTrain, 0.3, 3));

    //final JoystickButton shootAndFeedButton = new JoystickButton(driverController, Constants.shootAndFeedButton);
    //shootAndFeedButton.whenPressed(new ParallelShootAndFeed(shooter, feeder));

    final JoystickButton intakeButton = new JoystickButton(driverController, Constants.intakeButton);
    intakeButton.whileHeld(new SuckBalls(intake));

    //final JoystickButton shootButton = new JoystickButton(driverController, Constants.shootButton);
    //shootButton.whileHeld(new Shoot(shooter));
  }
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
      Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics, 10); 
      //max voltage is 10, so voltage is the same regardless of current battery voltage because the bat voltage is always >10

    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .setReversed(false);

    /*Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
      new Pose2d(1.3, -2.2, new Rotation2d(0)),
      List.of(
        //Pass through these two interior waypoints, making an 's' curve path
        //input as (y, -x) if you're thinking of the field as a normal Cartesian plane
        //new Translation2d(1, 1),
        //new Translation2d(2, -1)
        //new Translation2d(0.5, 0.75),
        //new Translation2d(1.0, 1.5),
        //new Translation2d(1.5, 0.75)
        new Translation2d(2.4, -2.1),
        //new Translation2d(2.9, -2.1),
        //new Translation2d(3.4, -2.2),
        new Translation2d(3.76, -2.27),
        //new Translation2d(4.05, -2.49),
        //new Translation2d(4.18, -2.68),
        //new Translation2d(4.29, -2.89),
        //new Translation2d(4.35, -3.15),
        //new Translation2d(4.31, -3.40),
        //new Translation2d(4.13, -3.48),
        new Translation2d(3.83, -3.53),
        //new Translation2d(3.52, -3.49),
        //new Translation2d(3.34, -3.26),
        //new Translation2d(3.38, -2.99),
        new Translation2d(3.51, -2.77),
        //new Translation2d(3.91, -2.50),
        new Translation2d(4.26, -2.42),
        new Translation2d(4.93, -2.37),
        new Translation2d(5.59, -2.28)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(5.82, -2.21, new Rotation2d(-0.3)),
      // Pass config
      config
    );*/

    
    //String trajectoryJSON = "paths/Unnamed.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/Paths/Test.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/output/Test.wpilib.json";
    //Path testPath = Filesystem.getDeployDirectory().toPath().resolve(Robot.trajectoryJSON);
    //Trajectory testTrajectory = exampleTrajectory; //new Trajectory(Trajectory.State(1., 1., 1., new Pose2d(0, 0, new Rotation2d(0)), 1.));

    /*RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
        double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        }
    };*/
    /*Trajectory exampleTrajectory1 = Robot.testTrajectory1;

    PIDController leftController = new PIDController(Constants.kPDriveVel, 0, 0);
    PIDController rightController = new PIDController(Constants.kPDriveVel, 0, 0);
    //PIDController leftController = new PIDController(0, 0, 0);
    //PIDController rightController = new PIDController(0, 0, 0);

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory1,
      driveTrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      //disabledRamsete,
      new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      driveTrain::getWheelSpeeds,
      leftController,
      rightController,
      // RamseteCommand passes volts to the callback
      //driveTrain::tankDriveVolts,
      (leftVolts, rightVolts) -> {
        driveTrain.tankDriveVolts(leftVolts, rightVolts);

        SmartDashboard.putNumber("left measurement", driveTrain.getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("left reference", leftController.getSetpoint());

        SmartDashboard.putNumber("right measurement", driveTrain.getWheelSpeeds().rightMetersPerSecond);
        SmartDashboard.putNumber("right reference", rightController.getSetpoint());
      },
      driveTrain
    );

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    //return ramseteCommand.alongWith(new SuckBalls(intake));
  
    //return chooser.getSelected(); 
    //return backwardsTrajectoryTest;*/
    return getForwardTestCommand()
    //.andThen(new InstantCommand(driveTrain::resetEncoders, driveTrain))
    //.andThen(getReverseTestCommand())
    //.andThen(new InstantCommand(driveTrain::resetEncoders, driveTrain))
    ;

  } 

  public Command getForwardTestCommand() {
    Trajectory exampleTrajectory1 = Robot.testTrajectory1;

    PIDController leftController = new PIDController(Constants.kPDriveVel, 0, 0);
    PIDController rightController = new PIDController(Constants.kPDriveVel, 0, 0);
    //PIDController leftController = new PIDController(0, 0, 0);
    //PIDController rightController = new PIDController(0, 0, 0);

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory1,
      driveTrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      //disabledRamsete,
      new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      driveTrain::getWheelSpeeds,
      leftController,
      rightController,
      // RamseteCommand passes volts to the callback
      //driveTrain::tankDriveVolts,
      (leftVolts, rightVolts) -> {
        driveTrain.tankDriveVolts(leftVolts, rightVolts);

        /*SmartDashboard.putNumber("left measurement", driveTrain.getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("left reference", leftController.getSetpoint());

        SmartDashboard.putNumber("right measurement", driveTrain.getWheelSpeeds().rightMetersPerSecond);
        SmartDashboard.putNumber("right reference", rightController.getSetpoint());*/
      },
      driveTrain
    );

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    //return ramseteCommand.alongWith(new SuckBalls(intake));
    //return new InstantCommand(driveTrain::resetEncoders, driveTrain).andThen()
  
  }
  
  public Command getReverseTestCommand() {
    Trajectory exampleTrajectory2 = Robot.testTrajectory2;

    PIDController leftController = new PIDController(Constants.kPDriveVel, 0, 0);
    PIDController rightController = new PIDController(Constants.kPDriveVel, 0, 0);
    //PIDController leftController = new PIDController(0, 0, 0);
    //PIDController rightController = new PIDController(0, 0, 0);

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory2,
      driveTrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      //disabledRamsete,
      new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      driveTrain::getWheelSpeeds,
      leftController,
      rightController,
      // RamseteCommand passes volts to the callback
      //driveTrain::tankDriveVolts,
      (leftVolts, rightVolts) -> {
        driveTrain.tankDriveVolts(leftVolts, rightVolts);

        /*SmartDashboard.putNumber("left measurement", driveTrain.getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("left reference", leftController.getSetpoint());

        SmartDashboard.putNumber("right measurement", driveTrain.getWheelSpeeds().rightMetersPerSecond);
        SmartDashboard.putNumber("right reference", rightController.getSetpoint());*/
      },
      driveTrain
    );

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory2.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    //return ramseteCommand.alongWith(new SuckBalls(intake));
  
  }
}
