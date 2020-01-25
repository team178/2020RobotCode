/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  private static VictorSPX elevator;
  private static VictorSPX winchMaster;
  private static VictorSPX winchSlave;
  private static VictorSPX leveler;

  public Climber() {
    elevator = new VictorSPX(RobotMap.elevator);
    winchMaster = new VictorSPX(RobotMap.winch1);
    winchSlave = new VictorSPX(RobotMap.winch2);
    leveler = new VictorSPX(RobotMap.leveler);
    winchSlave.set(ControlMode.Follower, RobotMap.winch1);
  }

  public void climb (double speed) {
    elevator.set(ControlMode.PercentOutput, speed);
  }

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
