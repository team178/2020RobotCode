/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class MoveToAngle extends CommandBase {
    private DriveTrain drivetrain;
    private static double currentAngle;
    private static double desiredAngle;
    private static final double tolerance = 10;
  
    public MoveToAngle(double angle) {
        addRequirements(Robot.driveTrain);
        desiredAngle = angle;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      drivetrain = Robot.driveTrain;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //This will move to the desired angle within a tolerance of 5  
    if(desiredAngle-10 < Robot.getCurrentAngle() + tolerance && desiredAngle-10 > Robot.getCurrentAngle() - tolerance) {
        drivetrain.drive(0, 0);
    } else {
        drivetrain.drive(-0.5, 0.5);
    }
    System.out.println(currentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(desiredAngle-10 < Robot.getCurrentAngle()+tolerance && desiredAngle-10 > Robot.getCurrentAngle()-tolerance);
  }
}
