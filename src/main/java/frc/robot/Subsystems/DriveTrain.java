/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {

  private final SPI.Port sPort = SPI.Port.kOnboardCS0;
  
  //DM & encoder declarations
  public static TalonSRX leftMaster;
  public static VictorSPX leftSlave;
  
  public static TalonSRX rightMaster;
  public static VictorSPX rightSlave;
  
  public static Encoder leftEncoder;
  public static Encoder rightEncoder;

  //gyro
  private final Gyro gyro = new ADXRS450_Gyro(sPort);
	
  //PID controllers
  private final PIDController leftPIDController = new PIDController(Constants.OPTIMAL_DRIVE_KP, 0, 0);
  private final PIDController rightPIDController = new PIDController(Constants.OPTIMAL_DRIVE_KP, 0, 0);

  //Autonomous path planning
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
 
  public DriveTrain() {
    //DM initializations
    leftMaster = new TalonSRX(RobotMap.DMTopLeft);
    leftSlave = new VictorSPX(RobotMap.DMBottomLeft);
    leftSlave.set(ControlMode.Follower, RobotMap.DMTopLeft);
    
    rightMaster = new TalonSRX(RobotMap.DMTopRight);
    rightSlave = new VictorSPX(RobotMap.DMBottomRight);
    rightSlave.set(ControlMode.Follower, RobotMap.DMTopRight);

    leftEncoder = new Encoder(RobotMap.Encoder1, RobotMap.Encoder2, false);
    rightEncoder = new Encoder(RobotMap.Encoder3, RobotMap.Encoder4, true);
    
    leftEncoder.setDistancePerPulse(Constants.ENCODER_DPP);
    rightEncoder.setDistancePerPulse(Constants.ENCODER_DPP);
  }

  public double getLeftDistance() {
      return leftEncoder.getDistance();
  }
  
  public double getRightDistance() {
      return rightEncoder.getDistance();
  }
  
  public void drive(double leftPower, double rightPower) {
    leftMaster.set(ControlMode.PercentOutput, -leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftMaster.set(ControlMode.PercentOutput, -leftVolts / 12);
    rightMaster.set(ControlMode.PercentOutput, rightVolts / 12);
  }
	
  public void driveVoltsBackwards(double leftVolts, double rightVolts) {
    leftMaster.set(ControlMode.PercentOutput, leftVolts / 12);
    rightMaster.set(ControlMode.PercentOutput, -rightVolts / 12);
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
	
  public PIDController getLeftPIDController() {
  	return leftPIDController;
  }
	
  public PIDController getRightPIDController() {
  	return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }
	
  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public double getGyroReading() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getLeftDistance(), getRightDistance());
  } 
}
