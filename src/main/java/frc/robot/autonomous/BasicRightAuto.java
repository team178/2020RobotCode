/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoBallDump;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.MoveToAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BasicRightAuto extends SequentialCommandGroup {
  /**
   * Creates a new BasicRightAuto.
   */
  public BasicRightAuto() {
  //  super(new AutoDrive(1, -3.116), new MoveToAngle(-60), new AutoBallDump(), new AutoDrive(1, -0.373));
  }
}
