/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  private static Solenoid elevator;
  private static TalonSRX winchMaster;
  private static TalonSRX winchSlave;
  private static TalonSRX leveler;

  public Climber() {
    elevator = new Solenoid(RobotMap.hookThurst);
    winchMaster = new TalonSRX(RobotMap.winchMaster);
    winchSlave = new TalonSRX(RobotMap.winchSlave);
    leveler = new TalonSRX(RobotMap.leveler);
    winchSlave.follow(winchMaster);
  }

  public void extendHook() {
    elevator.set(true);
  }

  public void retractHook() {
    elevator.set(false);
  }

  public void windWinch(double speed) {
    winchMaster.set(ControlMode.PercentOutput, speed);
  }

  public void moveAlongBar(double speed) {
    leveler.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    if (Robot.auxController.b.get()) {
      extendHook();
    }

    if (Robot.auxController.x.get()) {
      retractHook();
      windWinch(-1);
    }
  }
}
