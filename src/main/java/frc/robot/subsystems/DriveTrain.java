/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/*
  NOTES FOR NEW CODERS: 
  You should probably do the AP CSA CodeHS course before trying to code 
  for robotics. It was the most helpful course i've done, and the beginning is fun with Karel if you
  want to do it. Make sure you completely understand all the stuff about classes, objects, and methods.
  You can probably skip some of the string stuff, but it's still good to know.

  Once you have understand everything in the course, watch these series to apply your knowledge to FRC:
  1. FRC Java Programming Tutorials
  https://www.youtube.com/playlist?list=PLqolGlJdb9oWsgP4biujl_eTFazESWl8o 
  Note that this series is a little old, and there has been a new command based framework since then. 
  So don't copy his code at all. Just follow along with the logic of it.
  2. First Robotics Competition - Command Base System 2020 - VS Code - Java
  https://www.youtube.com/playlist?list=PLYwJIUT_B-n612Gqmfsq1ukYLa6WKgonc 
  This video uses the new framework.
  I would recommend that you follow along these videos without basing your code off of mine, especially
  if you encounter some errors. It is a learning experience.
  Good luck :) -Samuel, 2021
  
  
  FRC coding tips:
  1. Refrain from using wait() functions or while loops anywhere if possible cuz it can mess with the how the
  FRC scheduler and other stuff work and update. It might cause weird errors.
  2. If you have an error, someone has probably posted about it on Chief Delphi or Stack Overflow. 
  Google the error and see what pops up. It could also be a mechanical problem, especially if Pauley built it. 
  3. WPILIB has a lot of tutorials for various things on their website.
  
  Putting your code on GitHub: 
  1. Watch a few videos on how GitHub works - learn about pull requests, commits, pushes, branches, etc.
  2. Watch the GitHub VS Code tutorial video here:
  FRC - Command Base System - Java - VS Code 2020 - Part 10 by Nevin Morrison
  https://www.youtube.com/watch?v=goUt-VxCnE0&t=1s 
  3. It is good practice to commit after every day of work. Make sure code builds/deploys before commiting
  4. Write good commit messages
*/

//MAKING THIS CLASS WITH THE WPILIB TRAJECTORY TUTORIAL
public class DriveTrain extends SubsystemBase {
  //TO DO: SuppressWarnings class ??

  //TO DO: put tiny blacks screws and plastic cover back on talon. Uses 3/32 allen

  //TO DO: find whereever i used an instant command and change it to a lambda?
  //Also, graph pidController.getSetpoint() to test turn right command

  WPI_TalonSRX motorL1 = new WPI_TalonSRX(Constants2.MOTOR_L1.value);
  WPI_TalonSRX motorL2 = new WPI_TalonSRX(Constants.motorL2Value);
  WPI_TalonSRX motorR1 = new WPI_TalonSRX(Constants.motorR1Value);
  WPI_TalonSRX motorR2 = new WPI_TalonSRX(Constants.motorR2Value);

  private final SpeedControllerGroup leftMotors;
  private final SpeedControllerGroup rightMotors;

  //brake mode reduces wheel slip
  private final NeutralMode motorMode = NeutralMode.Brake;

  //private final DifferentialDrive diffDrive;

  private int driveReverser = 1;
  
  //encoder cycles per rev = pulse per rev
  //I am using type 1x here because the data is too noisy on the others
  private final Encoder leftDriveEnc = new Encoder(Constants.leftDriveEncChannelA, 
    Constants.leftDriveEncChannelB, false, CounterBase.EncodingType.k1X);
  private final Encoder rightDriveEnc = new Encoder(Constants.rightDriveEncChannelA, 
    Constants.rightDriveEncChannelB, true, CounterBase.EncodingType.k1X);

  //private final AnalogEncoder leftDriveEnc2 = new AnalogEncoder(new AnalogInput(0));
  //private final AnalogEncoder rightDriveEnc2 = new AnalogEncoder(new AnalogInput(3));

  /*
    Distance Per Pulse (dpp) calculation explanation:
    distance per pulse is pi * (wheel diameter / counts per revolution) according to Andymark enc example code.
    Using meters for wheel diam because tutorial said to.
    Counts per rev is 8192 for Rev Through Bore Encoder, but 2048 is the number that got the right measurement, so idk
  */
  private final double driveEncDPP = Math.PI * 0.1524 / 2048; //0.152400 meters is 6 inches
  //private final double driveEncDPP = 1 / 2048; //in this case the distance unit is one rotation i think

  //odometry class for tracking robot pose
  //pose is the robot's position and orientation
  //odometry is "the use of data from motion sensors to estimate change in position over time" -wikipedia
  private final DifferentialDriveOdometry odometry;

  /*TO DO: manually drive the auton course and log the encoder position/heading every 0.1 sec or so
  And then give the motors that velocity for the auton mode
  Then I wouldn't have to manually tune the trajectory
  Could made an array that holds the velocity and one that holds heading
  And then use a for:each loop to input it into ramsete?
  TrajectoryPoint class?
  */


  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

    motorL1.setNeutralMode(motorMode);
    motorL2.setNeutralMode(motorMode);
    motorR1.setNeutralMode(motorMode);
    motorR2.setNeutralMode(motorMode);

    motorL1.setSafetyEnabled(true);
    motorL2.setSafetyEnabled(true);
    motorR1.setSafetyEnabled(true);
    motorR2.setSafetyEnabled(true);

    //motorL1.setExpiration(30);
    //motorL2.setExpiration(30);
    //motorR1.setExpiration(10);
    //motorR2.setExpiration(10);

    //this is the deadzone for the motors. Any number below this get changed to zero.
    //Note that the minimum motor input to overcome static friction is about 0.0015 ish
    motorL1.configNeutralDeadband(0.001);
    motorL2.configNeutralDeadband(0.001);
    motorR1.configNeutralDeadband(0.001);
    motorR2.configNeutralDeadband(0.001);
    
    //TODO: play with this
    //motorL1.configPeakOutputForward(0.9);

    leftMotors = new SpeedControllerGroup(motorL1, motorL2);
    rightMotors = new SpeedControllerGroup(motorR1, motorR2);
    //diffDrive = new DifferentialDrive(leftMotors, rightMotors);

    leftDriveEnc.setDistancePerPulse(driveEncDPP);
    rightDriveEnc.setDistancePerPulse(driveEncDPP);

    //the samples to average thing makes the data less noisy
    //dont change number without testing the new number with the frc drive charaterization data plots
    leftDriveEnc.setSamplesToAverage(30);
    rightDriveEnc.setSamplesToAverage(30);

    //encoders have to be set to zero before constructing odometry class
    resetEncoders();
    NavX.zeroGyroYaw();
    
    odometry = new DifferentialDriveOdometry(NavX.getGyroRotation2d());
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TO DO: make this run faster? It's at 0.02s
    odometry.update(NavX.getGyroRotation2d(), leftDriveEnc.getDistance(), rightDriveEnc.getDistance());

    //var translation = odometry.getPoseMeters().getTranslation();
    //SmartDashboard.putNumber("translation x", translation.getX());
    //SmartDashboard.putNumber("translation y", translation.getY());
    //SmartDashboard.putNumber("rotation2d", odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("leftDriveEncDistance", leftDriveEnc.getDistance());
    SmartDashboard.putNumber("leftDriveEncRate", leftDriveEnc.getRate());
    SmartDashboard.putNumber("rightDriveEncDistance", rightDriveEnc.getDistance());
    SmartDashboard.putNumber("rightDriveEncRate", rightDriveEnc.getRate());
    SmartDashboard.putNumber("averageDriveEncDistance", getAverageEncDistance());

    SmartDashboard.putNumber("motorL1 output percent", motorL1.getMotorOutputPercent());

    //this is supposed to be so i dont get errors saying the motor output doesnt update enough,
    //but i still get them
    motorL1.feed();
    motorL2.feed();
    motorR1.feed();
    motorR2.feed();

  }

  /*
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    diffDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("zRotation", zRotation);
  }
  */

  public void setLeftMotors(double speed){
    motorL1.set(ControlMode.PercentOutput, speed * driveReverser);
    motorL2.set(ControlMode.PercentOutput, speed * driveReverser);
  }

  //right motors have inverted speed bc of how the motors are oriented on robot
  public void setRightMotors(double speed){
    motorR1.set(ControlMode.PercentOutput, -speed * driveReverser);
    motorR2.set(ControlMode.PercentOutput, -speed * driveReverser);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts * driveReverser);
    rightMotors.setVoltage(-rightVolts * driveReverser);

    motorL1.feed();
    motorL2.feed();
    motorR1.feed();
    motorR2.feed();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDriveEnc.getRate(), rightDriveEnc.getRate());
  }

  public void setMaxOutput(double maxOutput) {
    //diffDrive.setMaxOutput(maxOutput);
  }

  
  public double getLeftEncDistance() {
    return leftDriveEnc.getDistance();
  }

  public double getRightEncDistance() {
    return rightDriveEnc.getDistance();
  }

  public double getAverageEncDistance() {
    return (leftDriveEnc.getDistance() + rightDriveEnc.getDistance()) / 2;
  }

  public void stopDrive() {
    //motorL1.setNeutralMode(NeutralMode.Brake);
    //motorL2.setNeutralMode(NeutralMode.Brake);
    //motorR1.setNeutralMode(NeutralMode.Brake);
    //motorR2.setNeutralMode(NeutralMode.Brake);    
    
    motorL1.set(ControlMode.PercentOutput, 0);
    motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    motorR2.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncoders() {
    leftDriveEnc.reset();
    rightDriveEnc.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, NavX.getGyroRotation2d());
  }
  
  final double PIDDivisor = 1;
  public void usePIDOutput(double output) {
    SmartDashboard.putNumber("PIDOutput", output);
    setLeftMotors(output / PIDDivisor);
    setRightMotors(-output / PIDDivisor); //right is neg so it turns right
  }

  public void reverseDrive() {
    //motorL1.setInverted(true);
    //motorL2.setInverted(true);

    //driveReversed = (driveReversed == 1) ? 1 : -1;
    driveReverser = -1;

    leftDriveEnc.setReverseDirection(true);
    rightDriveEnc.setReverseDirection(false);

    //NavX.reverseGyro();
  }

  public void unReverseDrive() {
    driveReverser = 1;

    leftDriveEnc.setReverseDirection(false);
    rightDriveEnc.setReverseDirection(true);

    //NavX.unReverseGyro();
  }
}
