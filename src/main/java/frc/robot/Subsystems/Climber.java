/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  private static DoubleSolenoid elevator;
  private static VictorSPX winchMaster;
  private static VictorSPX winchSlave;
  private static VictorSPX leveler;

  public Climber() {
    elevator = new DoubleSolenoid(RobotMap.hookThurst1,RobotMap.hookThrust2);
    winchMaster = new VictorSPX(RobotMap.winchMaster);
    winchSlave = new VictorSPX(RobotMap.winchSlave);
    leveler = new VictorSPX(RobotMap.leveler);
    winchSlave.set(ControlMode.Follower, RobotMap.winchMaster);
  }

 // public void climb (double speed) {
    
 // }

  public void moveWinch (double speed) {
    winchMaster.set(ControlMode.PercentOutput, speed);
  }

  public void moveAlongBar (double speed) {
    leveler.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
