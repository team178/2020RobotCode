/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LawnMower;

public class AutoBallDump extends CommandBase {
  
  LawnMower lawnmower;
  
  public AutoBallDump() {
    addRequirements(Robot.lawnmower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lawnmower = Robot.lawnmower;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lawnmower.ballDump(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lawnmower.counter == 0;
  }
}
