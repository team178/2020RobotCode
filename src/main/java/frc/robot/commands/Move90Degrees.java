/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class Move90Degrees extends CommandBase {
  
  private OI oi;
  private DriveTrain drivetrain;
  private static double currentAngle;
  private static double increment = 90;
  private static final double tolerance = 5;

  public Move90Degrees() {
    addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      oi = Robot.oi;
      drivetrain = Robot.drivetrain;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
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
