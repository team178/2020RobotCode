/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.RobotMap;
/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {

  private final SPI.Port sPort = SPI.Port.kMXP;
  
  //DM & encoder declarations
  public static TalonSRX leftMaster;
  public static VictorSPX leftSlave;
  
  public static TalonSRX rightMaster;
  public static VictorSPX rightSlave;
  
  public static Encoder leftEncoder;
  public static Encoder rightEncoder;

  //gyro
  private final Gyro gyro = new ADXRS450_Gyro(sPort);

  //Autonomous path planning
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(PathConstants.kTrackWidthMeters);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
 
  public DriveTrain() {
    //Init DMs
	  leftMaster = new TalonSRX(RobotMap.DMLeftMaster);
	  leftSlave = new VictorSPX(RobotMap.DMLeftSlave);
	  rightMaster = new TalonSRX(RobotMap.DMRightMaster);
	  rightSlave = new VictorSPX(RobotMap.DMRightSlave);
	  
	  //Set victors to slaves
	  leftSlave.follow(leftMaster);
	  rightSlave.follow(rightMaster);
	  
	  //Config left motor & sensor directions
	  leftMaster.setInverted(true);
	  leftMaster.setSensorPhase(true);
	  leftSlave.setInverted(InvertType.FollowMaster);
	  
	  //Config right motor & sensor directions
	  rightMaster.setInverted(false);
	  rightMaster.setSensorPhase(false);
	  rightSlave.setInverted(InvertType.FollowMaster);

    //Config encoders (?)
    leftEncoder = new Encoder(RobotMap.Encoder1, RobotMap.Encoder2, false);
    rightEncoder = new Encoder(RobotMap.Encoder3, RobotMap.Encoder4, true);
    
    leftEncoder.setDistancePerPulse(PathConstants.kEncoderDPP);
    rightEncoder.setDistancePerPulse(PathConstants.kEncoderDPP);
  }

  public double getLeftDistance() {
      return leftEncoder.getDistance();
  }
  
  public double getRightDistance() {
      return rightEncoder.getDistance();
  }
  
  public void drive(double leftPower, double rightPower) {
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  /**
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  /**
   * @return the current robot pose according to odometry
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * @return the gyro heading (angle) as a Rotation2d object
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(0); //REPLACE WITH -(ACTUAL GYRO ANGLE)
  }

  /**
   * Reset encoders
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Reset gyro heading
   */
  public void resetHeading() {
    gyro.reset();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public double getGyroReading() {
    return -gyro.getAngle();
  }

  @Override
  public void periodic() {
    //Update odometry
    odometry.update(getHeading(), getLeftDistance(), getRightDistance());
  } 
}
