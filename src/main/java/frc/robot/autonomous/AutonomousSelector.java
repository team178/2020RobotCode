/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;


/**
 * Add your docs here.
 */
public class AutonomousSelector {

    private static DriveTrain driveTrain = Robot.drivetrain;
    
    public static Command getAutonomousCommand() {
        //Create trajectory config w/ parameters
        TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_VELOCITY_MPS, Constants.MAX_ACCEL_MPSPS)
            .setKinematics(driveTrain.getKinematics());

            //Nithin's test trajectory
        Trajectory nithinsTestTrajectory = TrajectoryGenerator.generateTrajectory(
            //Start pose
            new Pose2d(0, 0, new Rotation2d(0)),
            
            //Interior waypoints
            List.of(
                new Translation2d(1, 1),
                new Translation2d(-0.25, 4.5),
                new Translation2d(-3, 4),
                new Translation2d(-5, 7),
                new Translation2d(-5.5, 7.25)
            ),
            
            //End pose
            new Pose2d(-6, 8, new Rotation2d(53.14)),
            config
        );
        
        //Create ramsete command to drive path
        RamseteCommand ramseteCommand = new RamseteCommand(
            nithinsTestTrajectory,
            driveTrain::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            driveTrain.getFeedforward(),
            driveTrain.getKinematics(),
            driveTrain::getWheelSpeeds,
            driveTrain.getLeftPIDController(),
            driveTrain.getRightPIDController(),
            driveTrain::driveVolts,
            driveTrain
        );
        
        //Return ramsete command
        return ramseteCommand;
    }
}
