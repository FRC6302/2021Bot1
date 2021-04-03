/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //motors
	public static final int motorL1Value = 0;
	public static final int motorL2Value = 1;
	public static final int motorR1Value = 2;
    public static final int motorR2Value = 3;
    public static final int motorLeftLShooterValue = 5;
	public static final int motorLeftRShooterValue = 4;
	public static final int motorRightLShooterValue = 7;
    public static final int motorRightRShooterValue = 6;
    public static final int mecMotorL1Value = 90; //TO DO: change before running mecanum
	public static final int mecMotorL2Value = 91;
	public static final int mecMotorR1Value = 91;
	public static final int mecMotorR2Value = 93;
    public static final int motorIntakeValue = 11;
    
    //controller values
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int leftStickX = 0;
    public static final int leftStickY = 1;
    public static final int rightStickX = 4;
    public static final int rightStickY = 5;
    public static final int aButton = 1;
    public static final int bButton = 2;
    public static final int xButton = 3;
    public static final int yButton = 4;
    public static final int leftBumper = 5;
    public static final int rightBumper = 6;

    //controller ports (where they plug in at)
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

    //values obtained from frc drive characterization routine
    public static final double ksVolts = 0.978; //orig 1.08
    public static final double kvVoltSecondsPerMeter = 3.02; //orig 2.99
    public static final double kaVoltSecondsSquaredPerMeter = 0;//0.0561; //orig 0.0171
    public static final double kPDriveVel = 0.0133; // / 10; //orig 0.065
    public static final double kTrackwidthMeters = 0.578180; //orig 0.5778765
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    //values i stole from wpilib for testing
    //public static final double ksVolts = 0.22;
    //public static final double kvVoltSecondsPerMeter = 1.98;
    //public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    //public static final double kPDriveVel = 8.5;

    //drive encoders
    public static final int leftDriveEncChannelA = 7;
	public static final int leftDriveEncChannelB = 8;
	public static final int rightDriveEncChannelA = 1;
	public static final int rightDriveEncChannelB = 2;	
	
    //limelight
    public static final double limelightTXDeadzone = 0.5;
    public static final double limelightTargetXScaling = 1;
    public static final double limelightSpecificArea = 27;
    public static final double limelightTargetDistanceSpeed = 0.3;
    public static final double limelightSeekSpeed = 0.3;
    public static final double limelightTargetXSpeed = 0.2;
    public static final double limelightGetInRangeSpeed = 0.2;
    public static final double limelightTargetArea = 0.3;


    //intake
    public static final double intakeSpeed = 1.0;
    
    //Move command
    public static final int MoveTime = 1; 
    public static final double leftMotorsMoveSpeed = 0.3;
    public static final double rightMotorsMoveSpeed = 0.3;

    //shooter and feeder commands
    public static final double shootSpeed = 0.5;
    public static final double timedShootTime = 2;
    public static final double timedShootSpeed = 0.5; //was 1.0 last year

    public static final double FeedTime = 3;
    public static final double feederMotorMoveSpeed = 0.5;

    public static final int motorFeederValue = 0; //what is this??

    //gyro
    public static final double turnToZeroYawSpeed = 0.3;
    
    //miscellaneous
    public static final double turningRate = 0.5;
    public static final double deadzone = 0.4;
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    //"Reasonable baseline values for a RAMSETE follower in units of meters and seconds" -wpilib website
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //driver contoller buttons
    public static final int limelightTargetButton = Constants.aButton;
    public static final int moveButton = Constants.xButton;
    public static final int moveStraightButton = Constants.xButton;
	public static final int limelightGetInRangeButton = Constants.bButton;
	public static final int zeroYawButton = Constants.yButton;
	public static final int turnToYawZeroButton = Constants.rightBumper;
	public static final int turnRightButton = Constants.leftBumper;
	public static final int intakeButton = Constants.rightBumper;
	public static final int shootButton = Constants.bButton;
	
	
}
