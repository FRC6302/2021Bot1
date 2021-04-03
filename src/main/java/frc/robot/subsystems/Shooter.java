/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


//TO DO: COMBINE ALL THE SHOOTER AND FEEDER COMMANDS INTO ONE OR TWO USING LAMBDAS OR SOMETHING
public class Shooter extends SubsystemBase {
  TalonSRX motorLeftLShooter;
  TalonSRX motorLeftRShooter;
  TalonSRX motorRightLShooter;
  TalonSRX motorRightRShooter;
  
  TalonSRX motorFeeder;

  /*Encoder leftLEncoder;
  Encoder leftREncoder;
  Encoder rightLEncoder;
  Encoder rightREncoder;*/
  //MIGHT NEED TWO ENCODERS ONLY

  final double dpp = Math.PI * 6 / 1024; //blue wheel diam should be 6 inches
  /*
  Distance Per Pulse (dpp) calculation explanation:
  distance per pulse is pi * (wheel diameter / counts per revolution) according to Andymark example code
  counts per rev is 1024 for our encoder according to Andymark example code
  */
  //final double maxEncoderRate = ((18730 / (3.75 * 60)) * 6 * Math.PI * dpp); //around 28.9
  //TO DO: compare this to actual encoder output at max motor speed
  /*
  max encoder rate calculation explanation:
  gear ratio of motors is 30 to 8, or 3.75
  motor rpm is 18730 +/- 10%. Might need to change this num later
  60 seconds in a min
  18730 / (3.75 * 60) = 83.24 rps on the shooter wheels
  wheel circumference is 6 pi inches
  83.24 rps * 6 pi = 1569.12 inches per second at max power of shooter wheels
  I am multiplying 1569.12 * dpp (calculation above) because the documentation for the getRate()
    function says the rate returned is inches per second scaled by the dpp
  */

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    /*motorLeftLShooter = new TalonSRX(Constants.motorLeftLShooterValue);
    motorLeftRShooter = new TalonSRX(Constants.motorLeftRShooterValue);
    motorRightLShooter = new TalonSRX(Constants.motorRightLShooterValue);
    motorRightRShooter = new TalonSRX(Constants.motorRightRShooterValue);

    motorFeeder = new TalonSRX(Constants.motorFeederValue);

    leftLEncoder = new Encoder(90, 91); //I put random numbers on all these
    leftREncoder = new Encoder(92, 93); 
    rightLEncoder = new Encoder(94, 95); 
    rightREncoder = new Encoder(96, 97);*/
    //THESE NUMBERS ARE DIO PINS ON THE RIO IM PRETTY SURE. TO DO: FIGURE THIS OUT BEFORE RUNNING ENCODERS

    //distance per pulse is pi * (wheel diameter / counts per revolution) according to Andymark example code
    //counts per rev is 1024 for our encoder according to Andymark example code
    /*leftLEncoder.setDistancePerPulse(dpp);
    leftREncoder.setDistancePerPulse(dpp);
    rightLEncoder.setDistancePerPulse(dpp);
    rightREncoder.setDistancePerPulse(dpp);*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setShooterMotors(double speed){
    //left motors are negative so both wheels turn inward and shoot the ball
    motorLeftLShooter.set(ControlMode.PercentOutput, -speed);
    motorLeftRShooter.follow(motorLeftLShooter, FollowerType.PercentOutput);
    //motorLeftRShooter.set(ControlMode.PercentOutput, -speed);

    motorRightLShooter.set(ControlMode.PercentOutput, speed);
    motorRightRShooter.follow(motorRightLShooter, FollowerType.PercentOutput);
    //motorRightRShooter.set(ControlMode.PercentOutput, speed);

  }

  public void setShooterMotorsWith4Encoders(double speed, double getLeftLEncoderRate, double getLeftREncoderRate,
  double getRightLEncoderRate, double getRightREncoderRate){
    //pseudocode
    //get encoder values for all 4 motors
    //start them all out at the same speed input
    //if right master speed output is not with specific range of right follower, 
      //then adjust the speed of right follower somehow
    //do the same for left master and left follower, and then right master and left master
  }

  public void setShooterMotorsWith2Encoders(double speed){
    /*//pseudocode:
    //already have left and right followers following their master(their master's auxillary output?)
      //left motors are negative so both wheels turn inward and shoot the ball
    motorLeftLShooter.set(ControlMode.PercentOutput, -speed);
    motorLeftRShooter.follow(motorLeftLShooter, FollowerType.PercentOutput);
    motorRightLShooter.set(ControlMode.PercentOutput, speed);
    motorRightRShooter.follow(motorRightLShooter, FollowerType.PercentOutput);
    
    //set left master to desired speed
    motorLeftRShooter.set(ControlMode.PercentOutput, speed);
    
    //measure master outputs
    double leftREncoderRate = leftREncoder.getRate(); //inches per second times dpp?
    double rightLEncoderRate = rightLEncoder.getRate();

    //set right master based on measurement
    //these are the same. Combine them??
    if (rightLEncoderRate < leftREncoderRate * 0.95)
    {
      motorRightLShooter.set(ControlMode.PercentOutput, leftREncoderRate / maxEncoderRate);
    }
    else if (rightLEncoderRate > leftREncoderRate * 1.05)
    {
      motorRightLShooter.set(ControlMode.PercentOutput, leftREncoderRate / maxEncoderRate);
    }

    SmartDashboard.putNumber("leftREncoderRate", leftREncoderRate);
    SmartDashboard.putNumber("rightLEncoderRate", rightLEncoderRate);

    //if right master speed output is not with specific range of left master, 
      //then adjust the speed of right master somehow (with a calculation?)*/
  }

  public void setLeftShooterMotors(double speed){
    //left motors are negative so both wheels turn inward and shoot the ball
    motorLeftLShooter.set(ControlMode.PercentOutput, -speed);
    motorLeftRShooter.follow(motorLeftLShooter, FollowerType.PercentOutput);
    //motorLeftRShooter.set(ControlMode.PercentOutput, -speed);
  }

  public void setRightShooterMotors(double speed){
    motorRightLShooter.set(ControlMode.PercentOutput, speed);
    motorRightRShooter.follow(motorRightLShooter, FollowerType.PercentOutput);
    //motorRightRShooter.set(ControlMode.PercentOutput, speed);
  }


  public void stopShooter(){
    motorLeftLShooter.set(ControlMode.PercentOutput, 0);
    motorLeftRShooter.set(ControlMode.PercentOutput, 0);
    motorRightLShooter.set(ControlMode.PercentOutput, 0);
    motorRightRShooter.set(ControlMode.PercentOutput, 0);
  }

  public void setFeederMotor(double speed){
    motorFeeder.set(ControlMode.PercentOutput, speed);
  }

  public void stopFeeder(){
    motorFeeder.set(ControlMode.PercentOutput, 0);
  }

  //TO DO: COMBINE THESE GET METHODS SOMEHOW
  /*public double getLeftLEncoderRate(){
    return leftLEncoder.getRate(); //returns inches per second i believe cuz of using inches in dpp
  }

  public double getLeftREncoderRate(){
    return leftREncoder.getRate(); //returns inches per second i believe cuz of using inches in dpp
  }

  public double getRightLEncoderRate(){
    return rightLEncoder.getRate(); //returns inches per second i believe cuz of using inches in dpp
  }

  public double getRightREncoderRate(){
    return rightREncoder.getRate(); //returns inches per second i believe cuz of using inches in dpp
  }*/
}
