/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class LawnMower extends Subsystem {
  
  private static VictorSPX intake;
  private static DoubleSolenoid deployer;

  public LawnMower() {
    intake = new VictorSPX(RobotMap.intake);
    deployer = new DoubleSolenoid(RobotMap.deployerForward, RobotMap.deployerReverse);
  }

  public void intakeBall (double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void extendIntake () {
    deployer.set(DoubleSolenoid.Value.kForward); // Might be kReverse, test
  }

  public void retractIntake () {
    deployer.set(DoubleSolenoid.Value.kReverse); // Might be kForward, test
  }

  @Override
  protected void initDefaultCommand() {

  }
}
