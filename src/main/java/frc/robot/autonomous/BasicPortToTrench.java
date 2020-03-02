/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveToAngle;
import frc.robot.commands.AutoDrive;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BasicPortToTrench extends SequentialCommandGroup {
  
  public static double initDist = Robot.drivetrain.leftPosition.get();

  public BasicPortToTrench() {
    super(new MoveToAngle(22.4), new AutoDrive(0.5, initDist + 4.8));
}

}