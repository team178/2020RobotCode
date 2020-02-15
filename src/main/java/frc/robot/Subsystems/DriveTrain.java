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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

  //gyro
  private final Gyro gyro = new ADXRS450_Gyro(sPort);
 
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
  }

  public void drive(double leftPower, double rightPower) {
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
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
    double yReduction = Robot.mainController.trigger.get() ? 0.5 : 1;
    double twistReduction = Robot.mainController.trigger.get() ? 0.4 : 1;

    double yVal = Robot.mainController.getY() * yReduction;
    double twistVal = Robot.mainController.getTwist() * twistReduction;

    drive(yVal+twistVal, yVal-twistVal);
  } 
}
