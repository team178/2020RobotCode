/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Subsystems.DriveTrain;

public class TestDriveEncoders extends CommandBase {
  /**
   * Creates a new TestDriveEncoders.
   */
  private final DriveTrain driveTrain;

  public TestDriveEncoders() {
    driveTrain = Robot.drivetrain;
    //addRequirements(Robot.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.getLeftDistance();
    double tickPRotate = 1440;
    double wheelDiameter = 6;
    double wheelCirc = wheelDiameter * Math.PI;
    double tickConvert = wheelCirc/tickPRotate;
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
