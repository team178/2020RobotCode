/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive extends CommandBase {
  
  private DriveTrain driveTrain;
  private double speed;
  private double distance;
  
  public AutoDrive(double speed, double distance) {
    addRequirements(Robot.driveTrain);
    this.speed = speed;
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain = Robot.driveTrain;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(speed, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Math.abs(distance) - Math.abs(driveTrain.rightPosition.get())) < PathConstants.kDriveTolerance || 
    driveTrain.hasLeftCrashed.get() || driveTrain.hasRightCrashed.get();
  }
}
