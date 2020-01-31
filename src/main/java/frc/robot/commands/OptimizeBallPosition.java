/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LawnMower;
import frc.robot.OI;
import frc.robot.Robot;

public class OptimizeBallPosition extends CommandBase {
  
  LawnMower lawnmower;
  OI oi;
  double previousDetection1;
  double previousDetection2;
  public OptimizeBallPosition() {
     lawnmower = Robot.lawnmower;
     oi = Robot.oi;
     previousDetection1 = 0;
     previousDetection2 = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double speed = oi.getLeftTriggerAux();

  if (lawnmower.getTof1Distance() > 30 /* this value is yet to be tested, should be sensing the back wall of mechanism */) {
	  lawnmower.intakeBall(0);
  } else {
	  if (lawnmower.getTof2Distance() < 10 /* point at which the ball is sensed by the 2nd sensor where there is enough room for a new ball */) {
		  lawnmower.intakeBall(speed);
	  } else {
		  lawnmower.intakeBall(0);
	}
}

  previousDetection1 = lawnmower.getTof1Distance();
  previousDetection2 = lawnmower.getTof2Distance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
