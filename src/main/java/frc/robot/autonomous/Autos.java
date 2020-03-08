/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class Autos {
    //Basic autos
    public static SequentialCommandGroup BasicLeftAuto = new SequentialCommandGroup(new AutoDrive(-.5, 10), new AutoBallDump());
    public static SequentialCommandGroup BasicMiddleAuto = new SequentialCommandGroup(new AutoDrive(-.5, 9), new WaitCommand(0.2), new AutoBallDump());
    public static SequentialCommandGroup BasicRightAuto = new SequentialCommandGroup(new AutoDrive(1, -3.116), new MoveToAngle(-60), new AutoBallDump(), new AutoDrive(1, -0.373));
    
    //Paths
}
