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
import frc.robot.commands.Move90Degrees;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BasicAuto extends SequentialCommandGroup {
  /**
   * Creates a new BasicAuto.
   */
  public BasicAuto() {
    super(new AutoDrive(1, 2.743), new AutoBallDump(), new Move90Degrees(), new Move90Degrees(), new AutoDrive(1, 2.743));
  }
}
