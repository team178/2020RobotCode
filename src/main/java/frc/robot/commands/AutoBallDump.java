/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LawnMower;

public class AutoBallDump extends CommandBase {
  
  private LawnMower lawnMower;
  private double startTime;
  
  public AutoBallDump() {
    addRequirements(Robot.lawnmower);
    startTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lawnMower = Robot.lawnmower;
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lawnMower.ballDump(1, 0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lawnMower.ballDump(0, 0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lawnMower.counter == 0 || Timer.getFPGATimestamp() - startTime >= 3;
  }
}
