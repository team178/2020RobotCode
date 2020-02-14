/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class Move90Degrees extends CommandBase {
  
  private DriveTrain drivetrain;
  private static double increment = 90;
  private static final double tolerance = 5;

  public Move90Degrees() {
    addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      drivetrain = Robot.drivetrain;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(Robot.getCurrentAngle() > 270 && Robot.getCurrentAngle() < 360) {
          if(increment*4 < Robot.getCurrentAngle() + tolerance && increment*4 > Robot.getCurrentAngle() - tolerance) {
              drivetrain.drive(0, 0);
          } else {
              drivetrain.drive(-0.5, 0.5);
          }
      } else if(Robot.getCurrentAngle() > 180 && Robot.getCurrentAngle() < 270) {
        if(increment*3 < Robot.getCurrentAngle() + tolerance && increment*3 > Robot.getCurrentAngle() - tolerance) {
            drivetrain.drive(0, 0);
        } else {
            drivetrain.drive(-0.5, 0.5);
        }
      } else if(Robot.getCurrentAngle() > 90 && Robot.getCurrentAngle() < 180) {
        if(increment*2 < Robot.getCurrentAngle() + tolerance && increment*2 > Robot.getCurrentAngle() - tolerance) {
            drivetrain.drive(0, 0);
        } else {
            drivetrain.drive(-0.5, 0.5);
        }
      } else if(Robot.getCurrentAngle() > 0 && Robot.getCurrentAngle() < 90) {
        if(increment < Robot.getCurrentAngle() + tolerance && increment > Robot.getCurrentAngle() - tolerance) {
            drivetrain.drive(0, 0);
        } else {
            drivetrain.drive(-0.5, 0.5);
        }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( (Robot.getCurrentAngle() > 70 && Robot.getCurrentAngle() < 90)||(Robot.getCurrentAngle() > 160 && Robot.getCurrentAngle() < 180)||
    (Robot.getCurrentAngle() > 250 && Robot.getCurrentAngle() < 270)||(Robot.getCurrentAngle() > 340 && Robot.getCurrentAngle() < 360) );
  }
}
