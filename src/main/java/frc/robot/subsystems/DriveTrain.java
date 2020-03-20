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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {

  //Motor controllers
  private static WPI_TalonSRX leftMaster;
  private static WPI_VictorSPX leftSlave;
  private static WPI_TalonSRX rightMaster;
  private static WPI_VictorSPX rightSlave;

  private static SpeedControllerGroup leftMotors;
  private static SpeedControllerGroup rightMotors;

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
  
  //Gyro methods
  public Supplier<Double> headingDegrees;
  public Supplier<Rotation2d> headingRotation2d;

  //Other suppliers
  public Supplier<Double> leftCurrent;
  public Supplier<Double> rightCurrent;
  public Supplier<Boolean> hasLeftCrashed;
  public Supplier<Boolean> hasRightCrashed;

  //Misc
  private DriveDirection currentDirection = DriveDirection.INTAKE;
  
  public DriveTrain() {
    leftMaster = new WPI_TalonSRX(RobotMap.DMLeftMaster);
    leftSlave = new WPI_VictorSPX(RobotMap.DMLeftSlave);
    rightMaster = new WPI_TalonSRX(RobotMap.DMRightMaster);
    rightSlave = new WPI_VictorSPX(RobotMap.DMRightSlave);
    
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    
    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(false);
    
    leftPosition = () -> leftMaster.getSelectedSensorPosition(0) * PathConstants.kEncoderDPP; //r
    leftRate = () -> leftMaster.getSelectedSensorVelocity(0) * PathConstants.kEncoderDPP * 10; //r
    rightPosition = () -> rightMaster.getSelectedSensorPosition(0) * PathConstants.kEncoderDPP; //l
    rightRate = () -> rightMaster.getSelectedSensorVelocity(0) * PathConstants.kEncoderDPP * 10; //l
    
    headingDegrees = () -> -gyro.getAngle();
    headingRotation2d = () -> Rotation2d.fromDegrees(-gyro.getAngle());

    leftCurrent = () -> leftMaster.getSupplyCurrent();
    rightCurrent = () -> rightMaster.getSupplyCurrent();
    hasLeftCrashed = () -> leftCurrent.get() > 30;
    hasRightCrashed = () -> leftCurrent.get() > 30;
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMotors = new SpeedControllerGroup(leftMaster, leftSlave);
    rightMotors = new SpeedControllerGroup(leftMaster, leftSlave);
    
    kinematics = new DifferentialDriveKinematics(PathConstants.kTrackWidthMeters);
    odometry = new DifferentialDriveOdometry(getAngle());
    feedforward = new SimpleMotorFeedforward(PathConstants.kS, PathConstants.kV, PathConstants.kA);

    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);
    leftPIDController = new PIDController(PathConstants.kDriveP, 0, 0);

    leftMaster.setSafetyEnabled(false);
    rightMaster.setSafetyEnabled(false);
  }

  public void drive(double leftPower, double rightPower) {
    if (Math.abs(leftPower) < 0.1) {
      leftPower = 0;
    }
    if (Math.abs(rightPower) < 0.1) {
      rightPower = 0;
    }

    leftMaster.set(ControlMode.PercentOutput, currentDirection == DriveDirection.INTAKE ? leftPower : rightPower);
    rightMaster.set(ControlMode.PercentOutput, currentDirection == DriveDirection.INTAKE ? rightPower : leftPower);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public void toggleDriveDirection() {
    currentDirection = currentDirection == DriveDirection.INTAKE ? DriveDirection.SHOOTER : DriveDirection.INTAKE;
  }
  
  public void setDriveDirection(DriveDirection requestedDirection) {
    currentDirection = requestedDirection;
  }
  
  public DriveDirection getCurrentDriveDirection() {
    return currentDirection;
  }

  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftRate.get(),
      rightRate.get()
    );
  }

  /*
   * Gyro methods
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
   * Odometry methods
   */
  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getAngle());
  }

  @Override
  public void periodic() {
    if (RobotState.isOperatorControl()) {
      //Joystick drive
      yReduction = Robot.mainController.trigger.get() ? 0.5 : 1;
      twistReduction = Robot.mainController.trigger.get() ? 0.2 : 0.3;
      
      yVal = Robot.mainController.getY() * yReduction;
      twistVal = Robot.mainController.getTwist() * twistReduction;

      //wiimote drive cause vivek said it would be a good idea to test it on the main bot
      //yReduction = Robot.wiiRemote.b.get() ? 0.5 : 1;
      //twistReduction = Robot.wiiRemote.b.get() ? 0.2 : 0.5;
      
      //yVal = -Robot.wiiRemote.getYRot();
      //twistVal = Robot.wiiRemote.getXRot();

      drive(twistVal+yVal, twistVal-yVal);
    }

    //Path planning
    odometry.update(getAngle(), leftPosition.get(), rightPosition.get());
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
    INTAKE,
    SHOOTER
  }
}
