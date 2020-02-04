/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.commands;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.WheelOfFortuneContestant;

import frc.robot.ColorSensor;

public class SpinWheelOfFortuneByRotations extends CommandBase {
 

  private static WheelOfFortuneContestant wheelOfFortuneContestant = Robot.wheeloffortunecontestant;
  public static final double spinPower = 1;
  

  public SpinWheelOfFortuneByRotations() {
    addRequirements(Robot.wheeloffortunecontestant);
  }

  // Called when the command is initially scheduled.
 
  @Override
  public void initialize() { //init wheeloffortunecontestant subsystem
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //spins the wheel

    while(wheelOfFortuneContestant.getRot() < 3)
    {
      wheelOfFortuneContestant.spinToWin(spinPower);
    }
    
      wheelOfFortuneContestant.spinToWin(0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //if rotations between 3-5 return true
    return true;
  }
}
*/