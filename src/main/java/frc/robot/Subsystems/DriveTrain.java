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
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  
  //DM declarations
  public static TalonSRX left1;
  public static VictorSPX left2;
  public static TalonSRX right1;
  public static VictorSPX right2;
  public static Encoder left; 
  public static Encoder right;
 
  public DriveTrain() {
	  //DM initializations
	  left1 = new TalonSRX(RobotMap.DMTopLeft);
	  left2 = new VictorSPX(RobotMap.DMBottomLeft);
	  right1 = new TalonSRX(RobotMap.DMTopRight);
      right2 = new VictorSPX(RobotMap.DMBottomRight);
      left = new Encoder(RobotMap.Encoder1, RobotMap.Encoder2);
      right = new Encoder(RobotMap.Encoder3, RobotMap.Encoder4);


  }

  public int inchesToTicks(double inches) {
    double tickPRotate = 1440;
    double wheelDiameter = 6;
    // the encoder wheel is in inches
    double wheelCirc = wheelDiameter * Math.PI;

    double rotations = inches / wheelCirc;
    return (int) (rotations * tickPRotate);
  }

  public double getLeftDistance()
  {
      return left.getDistance();
  }
  
  public double getRightDistance()
  {
      return right.getDistance();
  }
  
  
  public void drive(final double leftPower, final double rightPower) {
    left1.set(ControlMode.PercentOutput, -leftPower);
    left2.set(ControlMode.Follower, RobotMap.DMTopLeft);
    right1.set(ControlMode.PercentOutput, rightPower);
    right2.set(ControlMode.Follower, RobotMap.DMTopRight);
  }

  @Override
  public void initDefaultCommand() {

  }
}
