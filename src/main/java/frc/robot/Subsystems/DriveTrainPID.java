/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.RobotMap;

public class DriveTrainPID extends PIDSubsystem {
  
  public static TalonSRX leftMaster;
  public static VictorSPX leftSlave; 
  public static TalonSRX rightMaster;
  public static VictorSPX rightSlave;
  
  public static Encoder leftEncoder;
  public static Encoder rightEncoder;

  private final SPI.Port sPort = SPI.Port.kOnboardCS0;
  final ADXRS450_Gyro gyro = new ADXRS450_Gyro(sPort);

  public DriveTrainPID() {
    super(new PIDController(0, 0, 0)); //We need to test and refine these values

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

  @Override
  public void useOutput(double output, double setpoint) {
      leftMaster.set(ControlMode.PercentOutput, -output);
      rightMaster.set(ControlMode.PercentOutput, output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return gyro.getAngle();
  }
}
