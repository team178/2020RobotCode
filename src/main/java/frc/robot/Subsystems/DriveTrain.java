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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  
  //DM declarations
  public static TalonSRX left1;
  public static VictorSPX left2;
  public static TalonSRX right1;
  public static VictorSPX right2;
  public static Encoder enc; 
 
  public DriveTrain() {
	  //DM initializations
	  left1 = new TalonSRX(RobotMap.DMTopLeft);
	  left2 = new VictorSPX(RobotMap.DMBottomLeft);
	  right1 = new TalonSRX(RobotMap.DMTopRight);
      right2 = new VictorSPX(RobotMap.DMBottomRight);
      enc = new Encoder(RobotMap.Encoder1, RobotMap.Encoder2);


  }
  
  public void drive(double leftPower, double rightPower) {
    left1.set(ControlMode.PercentOutput, -leftPower);
    left2.set(ControlMode.PercentOutput, -leftPower);
    right1.set(ControlMode.PercentOutput, rightPower);
    right2.set(ControlMode.PercentOutput, rightPower);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }
}
