/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoBallDump;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoLeftShoot extends SequentialCommandGroup {
  /**
   * Creates a new AutoLeftShoot.
   */
  public AutoLeftShoot() {
  //  super(PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryLeftForward),
  //  new AutoBallDump(),
  //  PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryLeftBack));
  }
}
