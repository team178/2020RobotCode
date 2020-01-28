/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class AutonomousSelector {

    private static DriveTrain driveTrain = Robot.drivetrain;
    
    public static Command getAutonomousCommand(boolean presetMode) {
        TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_VELOCITY_MPS, Constants.MAX_ACCEL_MPSPS)
            .setKinematics(driveTrain.getKinematics());
        Trajectory trajetory;
        if (presetMode) {
            trajetory = TrajectoryGenerator.generateTrajectory(initial, interiorWaypoints, end, config);
        } else {
            trajetory = TrajectoryGenerator.generateTrajectory(initial, interiorWaypoints, end, config);
        }
    }
}
