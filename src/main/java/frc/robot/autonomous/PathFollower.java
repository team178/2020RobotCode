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
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveTrain;

public class PathFollower {

    private static DriveTrain driveTrain = Robot.driveTrain;

    public static Command trajectoryToRamseteCommand(Trajectory trajectory, boolean reversed) {
        //Creating autonomous voltage constraint
        DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
            driveTrain.getFeedforward(), 
            driveTrain.getKinematics(), 
            PathConstants.kMaxVoltage
        );

        //Creating trajectory config
        TrajectoryConfig config = new TrajectoryConfig(
            PathConstants.kMaxVelMPS, 
            PathConstants.kMaxAccelMPSPS
        )
        .setKinematics(driveTrain.getKinematics())
        .setReversed(reversed)
        .addConstraint(voltageConstraint);

        //Nithin's test trajectory -- temporary
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

        //Create a ramsete command based off RamseteController & params
        RamseteCommand ramseteCommand = new RamseteCommand(
            nithinsTestTrajectory, //replace with trajectory after testing
            driveTrain::getPoseMeters,
            new RamseteController(PathConstants.kRamseteB, PathConstants.kRamseteZeta),
            driveTrain.getFeedforward(),
            driveTrain.getKinematics(),
            driveTrain::getWheelSpeeds,
            driveTrain.getLeftPIDController(),
            driveTrain.getRightPIDController(),
            driveTrain::driveVolts,
            driveTrain
        );
        
        //Added crash detection in parallel w/ ramsete command to end path if robot crashes into a wall -- yet to be tested
        //return ramseteCommand.andThen(() -> driveTrain.driveVolts(0, 0));
        return (new ParallelRaceGroup(ramseteCommand, () -> driveTrain.parallelDetectCrash())).andThen(() -> driveTrain.driveVolts(0, 0));
    }

    public Trajectory loadTrajectoryFromPathWeaver(String trajectoryJSONPath) {
        Trajectory trajectory = null;
        try {
            Path filePath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(filePath);
        } catch (IOException e) {
            DriverStation.reportError("cant find ur path " + trajectoryJSONPath + " bud u have a null trajectory now", e.getStackTrace());
        }
        return trajectory;
    }
}
