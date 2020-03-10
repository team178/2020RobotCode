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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  private static Solenoid elevator;
  private static VictorSPX winchMaster;
  private static VictorSPX winchSlave;
  private static TalonSRX leveler;

  private boolean enableClimber = false;

  public Climber() {
    elevator = new Solenoid(RobotMap.hookThurst);
    winchMaster = new VictorSPX(RobotMap.winchMaster);
    winchSlave = new VictorSPX(RobotMap.winchSlave);
  }

  public void extendHook(boolean doubleButtonRestrict) {
    if (enableClimber) {
      if (doubleButtonRestrict) {
        if (Robot.mainController.leftPadTop1.get()) {
          elevator.set(true);
        }
      } else {
        elevator.set(true);
      } 
    }
  }

  public void retractHook() {
    elevator.set(false);
  }

  public void windWinch(double speed) {
    winchMaster.set(ControlMode.PercentOutput, speed);
    winchSlave.set(ControlMode.PercentOutput, speed);
  }

  public void moveAlongBar(double speed) {
    leveler.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    //Climbing controls for aux
    if (Robot.auxController.getRightTrigger() > 0) {
      if (elevator.get()) {
        retractHook();
      }
      windWinch(Robot.auxController.getRightTrigger());
    }
    
    //Endgame timer restriction
    if (Timer.getMatchTime() < 29) {
      enableClimber = true;
    }
  }
}
