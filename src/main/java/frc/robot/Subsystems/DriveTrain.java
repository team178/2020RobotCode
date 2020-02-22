/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

  //Motor controllers
  private static WPI_TalonSRX leftMaster;
  private static WPI_VictorSPX leftSlave;
  private static WPI_TalonSRX rightMaster;
  private static WPI_VictorSPX rightSlave;

  private static SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMaster, leftSlave);
  private static SpeedControllerGroup rightMotors = new SpeedControllerGroup(leftMaster, leftSlave);

  //Gyro
  private final Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  //Pathplanning
  private static DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;
  private static SimpleMotorFeedforward feedforward;

  private static PIDController leftPIDController;
  private static PIDController rightPIDController;

  private static double yVal;
  private static double twistVal;
  private static double yReduction;
  private static double twistReduction;

  //Encoder methods
  public Supplier<Double> leftPosition;
  public Supplier<Double> rightPosition;
  public Supplier<Double> leftRate;
  public Supplier<Double> rightRate;
  
  //Gyro mehtods
  public Supplier<Double> headingDegrees;
  public Supplier<Rotation2d> headingRotation2d;
  
  public DriveTrain() {
    leftMaster = new WPI_TalonSRX(RobotMap.DMLeftMaster);
    leftSlave = new WPI_VictorSPX(RobotMap.DMLeftSlave);
    rightMaster = new WPI_TalonSRX(RobotMap.DMRightMaster);
    rightSlave = new WPI_VictorSPX(RobotMap.DMRightSlave);
    
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);

    leftPosition = () -> leftMaster.getSelectedSensorPosition(0) * PathConstants.kEncoderDPP;
    leftRate = () -> leftMaster.getSelectedSensorVelocity(0) * PathConstants.kEncoderDPP * 10;
    rightPosition = () -> rightMaster.getSelectedSensorPosition(0) * PathConstants.kEncoderDPP;
    rightRate = () -> rightMaster.getSelectedSensorVelocity(0) * PathConstants.kEncoderDPP * 10;
    
    headingDegrees = () -> -gyro.getAngle();
    headingRotation2d = () -> Rotation2d.fromDegrees(-gyro.getAngle());
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    setDriveDirection(DriveDirection.FORWARD);

    kinematics = new DifferentialDriveKinematics(PathConstants.kTrackWidthMeters);
    odometry = new DifferentialDriveOdometry(getAngle());
    feedforward = new SimpleMotorFeedforward(PathConstants.kS, PathConstants.kV, PathConstants.kA);

    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);
    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);
  }

  public void drive(double leftPower, double rightPower) {
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public void setDriveDirection(DriveDirection driveDirection) {
    if (driveDirection == DriveDirection.FORWARD) {
      leftMaster.setInverted(false);
      rightMaster.setInverted(true);
    } else {
      leftMaster.setInverted(true);
      rightMaster.setInverted(false);
    }
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftRate.get(),
      rightRate.get()
    );
  }

  /*
   * gyro methods
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
  
  public void calibrateGyro() {
    gyro.calibrate();
  }

  public void resetGyro() {
    gyro.reset(); 
  }

  /*
   * odometry methods
   */
  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getAngle());
  }

  @Override
  public void periodic() {
    //Joystick drive
    yReduction = Robot.mainController.trigger.get() ? 0.5 : 1;
    twistReduction = Robot.mainController.trigger.get() ? 0.4 : 1;

    yVal = Robot.mainController.getY() * yReduction;
    twistVal = Robot.mainController.getTwist() * twistReduction;

    drive(yVal+twistVal, yVal-twistVal);

    //Path planning
    odometry.update(getAngle(), leftRate.get(), rightRate.get());
  }

  public double getGyroReading() {
    return gyro.getAngle();
  }

  /*
   * path planning accessors
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  enum DriveDirection {
    FORWARD,
    BACKWARD
  }
}
