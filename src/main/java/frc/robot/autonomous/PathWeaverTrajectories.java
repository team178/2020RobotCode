/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.Constants.PathConstants;

/**
 * Add your docs here.
 */
    public class PathWeaverTrajectories {

        public static String BlueLeftToPort = "Paths/BlueLeftToPort.json";
        public static String BlueMiddleToPort = "Paths/BlueMiddleToPort.json";
        public static String BlueRightToPort = "Paths/BlueRightToPort.json";
        public static String RedLeftToPort = "Paths/RedLeftToPort.json";
        public static String RedMiddleToPort = "Paths/RedMiddleToPort.json";
        public static String RedRightToPort = "Paths/RedRightToPort.json";

        public static String BluePortToTrench = "Paths/BluePortToTrench.json";
        public static String RedPortToTrench = "Paths/RedPortToTrench.json";

        public static String[] BlueTrajectories = {BlueLeftToPort, BlueMiddleToPort, BlueRightToPort, BluePortToTrench};
        public static String[] RedTrajectories = {RedLeftToPort, RedMiddleToPort, RedRightToPort, RedPortToTrench};

        public static Trajectory defaultTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(0,0)
            ),
            new Pose2d(0, 0, new Rotation2d(0)),
            new TrajectoryConfig(
            PathConstants.kMaxVelMPS, 
            PathConstants.kMaxAccelMPSPS
        )
        .setKinematics(Robot.driveTrain.getKinematics())
        .setReversed(false)
        .addConstraint(new DifferentialDriveVoltageConstraint(
            Robot.driveTrain.getFeedforward(), 
            Robot.driveTrain.getKinematics(), 
            PathConstants.kMaxVoltage)
        ));

    public static Trajectory createTrajectory (String pathFile) {
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve(pathFile);
            return TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open Trajectory: " + pathFile, e.getStackTrace());
            return defaultTrajectory;
        }
        
    }
        
    public static RamseteCommand getRamseteCommand(Trajectory path) {
        RamseteCommand getCommand = new RamseteCommand(
            path,
            Robot.driveTrain::getPoseMeters,
            new RamseteController(PathConstants.kRamseteB, PathConstants.kRamseteZeta),
            Robot.driveTrain.getFeedforward(),
            Robot.driveTrain.getKinematics(),
            Robot.driveTrain::getWheelSpeeds,
            Robot.driveTrain.getLeftPIDController(),
            Robot.driveTrain.getRightPIDController(),
            Robot.driveTrain::driveVolts,
            Robot.driveTrain
        );
        return getCommand;
    }
}
