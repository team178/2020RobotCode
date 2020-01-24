/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  
  //DM & encoder declarations
  public static TalonSRX leftMaster;
  public static VictorSPX leftSlave; 
  public static TalonSRX rightMaster;
  public static VictorSPX rightSlave;
  
  public static Encoder leftEncoder;
  public static Encoder rightEncoder;

  //Autonomous path planning
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
 
  public DriveTrain() {
	  //DM initializations
	  leftMaster = new TalonSRX(RobotMap.DMTopLeft);
    leftSlave = new VictorSPX(RobotMap.DMBottomLeft);
    leftSlave.set(ControlMode.Follower, RobotMap.DMTopLeft);
    leftEncoder = new Encoder(RobotMap.Encoder1, RobotMap.Encoder2);
    leftMaster.setInverted(true);

	  rightMaster = new TalonSRX(RobotMap.DMTopRight);
    rightSlave = new VictorSPX(RobotMap.DMBottomRight);
    rightSlave.set(ControlMode.Follower, RobotMap.DMTopRight);
    rightEncoder = new Encoder(RobotMap.Encoder3, RobotMap.Encoder4);
    rightMaster.setInverted(false);
  }

  public double getLeftDistance() {
      return leftEncoder.getDistance();
  }
  
  public double getRightDistance() {
      return rightEncoder.getDistance();
  }
  
  public void drive(final double leftPower, final double rightPower) {
    leftMaster.set(ControlMode.PercentOutput, -leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  /**
   * @return the gyro heading (angle) as a Rotation2d object
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(0); //REPLACE WITH -(ACTUAL GYRO ANGLE)
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftEncoder.getRate() / 60 * Units.inchesToMeters(Constants.WHEEL_CIRCUMFRENCE),
      rightEncoder.getRate() / 60 * Units.inchesToMeters(Constants.WHEEL_CIRCUMFRENCE)
    );
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getLeftDistance(), getRightDistance());
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub
  }
}
