/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.PossibleTrajectories;
import frc.robot.subsystems.Climber;
//import frc.robot.autonomous.AutonomousSelector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LawnMower;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.WheelOfFortuneContestant;
import frc.robot.subsystems.LightsArduino;
import libs.IO.ThrustmasterJoystick;
import libs.IO.XboxController;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project....
 */
public class Robot extends TimedRobot {
  
  //Declare auto sendable choosers
  public static SendableChooser<Integer> robotDelayInt = new SendableChooser<>();
  public static SendableChooser<Integer> robotDelayDec = new SendableChooser<>();
  public static SendableChooser<String> startPath = new SendableChooser<>();
  public static SendableChooser<String> endLocation = new SendableChooser<>();
  
  // Declare subsystems
  public static DriveTrain driveTrain = new DriveTrain();
  public static LawnMower lawnMower = new LawnMower();
  public static WheelOfFortuneContestant wheelOfFortuneContestant = new WheelOfFortuneContestant();
  public static LightsArduino lights = new LightsArduino(Port.kOnboard, RobotMap.lightsI2CAddress);
  public static LightStrip lightStrip = new LightStrip(RobotMap.lightsPWM, RobotMap.numOfLEDs);
  public static Climber climber = new Climber();

  public static SubsystemBase[] subsystems = {
    driveTrain,
    lawnMower,
    wheelOfFortuneContestant,
    lights,
    lightStrip,
    climber
  };
  
  public static String gameData;
  public static double tof1Previous;
  public static double tof2Previous;
  private static double currentAngle;
  
  //Declare joysticks
  public static ThrustmasterJoystick mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
  public static XboxController auxController = new XboxController(RobotMap.JoystickPortXBoxAux);
  
  //Declare autonomous command
  //private Command autonomousCommand;

  //USB Camera declarations
  public static CameraServer camserv;
  public static UsbCamera camera;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    //Creates options for robot start path drop down
    startPath.addOption("Left","Left");
    startPath.addOption("Middle","Middle");
    startPath.addOption("Right","Right");

    //Creates options for robot end location drop down
    endLocation.addOption("Left","Left");
    endLocation.addOption("Middle","Middle");
    endLocation.addOption("Right","Right");

    //Creates options for robot timer integer numbers
    robotDelayInt.addOption("Integer Number for Timer", 0);
    robotDelayInt.addOption("Integer Number for Timer", 1);
    robotDelayInt.addOption("Integer Number for Timer", 2);
    robotDelayInt.addOption("Integer Number for Timer", 3);
    robotDelayInt.addOption("Integer Number for Timer", 4);
    robotDelayInt.addOption("Integer Number for Timer", 5);
    robotDelayInt.addOption("Integer Number for Timer", 6);
    robotDelayInt.addOption("Integer Number for Timer", 7);
    robotDelayInt.addOption("Integer Number for Timer", 8);
    robotDelayInt.addOption("Integer Number for Timer", 9);
    robotDelayInt.addOption("Integer Number for Timer", 10);
    robotDelayInt.addOption("Integer Number for Timer", 11);
    robotDelayInt.addOption("Integer Number for Timer", 12);
    robotDelayInt.addOption("Integer Number for Timer", 13);
    robotDelayInt.addOption("Integer Number for Timer", 14);
    robotDelayInt.addOption("Integer Number for Timer", 15);

    //Creates options for robot timer decimal part of integer
    robotDelayDec.addOption("Decimal part of timer", 0);
    robotDelayDec.addOption("Decimal part of timer", 1);
    robotDelayDec.addOption("Decimal part of timer", 2);
    robotDelayDec.addOption("Decimal part of timer", 3);
    robotDelayDec.addOption("Decimal part of timer", 4);
    robotDelayDec.addOption("Decimal part of timer", 5);
    robotDelayDec.addOption("Decimal part of timer", 6);
    robotDelayDec.addOption("Decimal part of timer", 7);
    robotDelayDec.addOption("Decimal part of timer", 8);
    robotDelayDec.addOption("Decimal part of timer", 9);

    SmartDashboard.putData("AutoLocation", startPath);
    SmartDashboard.putData("AutoLocation", endLocation);
    
    driveTrain.calibrateGyro();
    gameData = "";
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    //Camera initializations
    camserv = CameraServer.getInstance();
    
    //Camera 1
    camera = camserv.startAutomaticCapture("cam1", 0);
    //camera.setResolution(160, 90);
    camera.setFPS(14);
    camera.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras
  }
  public void changeCamera(String newName, int newPort) {
    camera = camserv.startAutomaticCapture(newName, newPort);
    camera.setFPS(14);
    camera.setPixelFormat(PixelFormat.kYUYV);
  }
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //camera stuff
    if (mainController.headLeft.get()) {
      changeCamera("cam0", 0);
    }

    if (mainController.headBottom.get()) {
      changeCamera("cam1", 1);
    }

    if (mainController.headRight.get()) {
      changeCamera("cam2", 2);
    }
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    SmartDashboard.putNumber("Gyro Reading", driveTrain.getAngle().getDegrees());
    SmartDashboard.putNumber("Balls in Lawn Mower", lawnMower.getCounter());

    //Gyro stuff
    if(driveTrain.getAngle().getDegrees()%360 == 0)
    {
      currentAngle = driveTrain.getAngle().getDegrees();
    } else {
      currentAngle = Math.abs(driveTrain.getAngle().getDegrees()%360);
    }
    System.out.println("Gyro Reading: " + driveTrain.getAngle());
    System.out.println("Current Angle Reading: " + currentAngle);

    climber.periodic();
    driveTrain.periodic();
    lights.periodic();
    wheelOfFortuneContestant.periodic();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  CommandScheduler.getInstance().run();
  double timerValue = 0;

  Trajectory sTrajectory = null;
  Trajectory eTrajectory = null;

  RamseteCommand rForward = null;
  RamseteCommand rBackward = null;

//the command rForward / rBackward refers to direction of the robot, either forwards or backwards;
//forward is used when the robot moves from its station into the dumpball area
//backward is used when the robot comes back from dumpball area into its station
  if (startPath.getSelected() != "" && endLocation.getSelected() != ""){
    timerValue = robotDelayInt.getSelected() + robotDelayDec.getSelected() / 10;

    String path1 = startPath.getSelected();
    String path2 = endLocation.getSelected();
//dictates that whenever path 1 is called, the robot is in its start phase of moving towards the balldump

//the trajectories refer to either s(tart) or e(nd); s refers to robot's movement towards the balldump and e to its return
    if (path1.equals("Left")) {
      sTrajectory = PossibleTrajectories.TrajectoryLeftForward;
    } else if (path1.equals("Middle")) {
      sTrajectory = PossibleTrajectories.TrajectoryMiddleForward;
    } else if (path1.equals("Right")) {
      sTrajectory = PossibleTrajectories.TrajectoryRightForward;
    }
//thus, if the robot is moving from its station into the balldump area, we choose which station it starts out from
    if (path2.equals("Left")) {
      eTrajectory = PossibleTrajectories.TrajectoryLeftBack;
    } else if (path2.equals("Middle")) {
      eTrajectory = PossibleTrajectories.TrajectoryMiddleBack;
    } else if (path2.equals("Right")) {
      eTrajectory = PossibleTrajectories.TrajectoryRightBack;
    }
//thus, if the robot is moving from the balldump area, we choose which station it will move into
    rForward = PossibleTrajectories.getRamseteCommand(sTrajectory);
    rBackward = PossibleTrajectories.getRamseteCommand(eTrajectory);
//dictates that robot will move (forward) at the (s)tart
// as well as robot will move (backward) at the (e)nd
  }
//the wait time is necessary in order to avoid other robots in our robot's path
  Command autonomousCommand = new WaitCommand(timerValue);
  autonomousCommand.andThen(rForward);
  autonomousCommand.andThen(() -> lawnMower.ballDump(1));
  autonomousCommand.andThen(rBackward);
//commented this out because the code was made more efficient by vivek. sorry yellow gang
// -- liza
  //Might scrw up if keep changing startPath and endLocation
  // if (startPath.getSelected() != "" && endLocation.getSelected() != "") {
  //   if (path1.equals("Left")) {
  //     autonomousCommand = autonomousCommand.andThen(PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryLeftForward);
  //     autonomousCommand = autonomousCommand.andThen(() -> lawnmower.ballDump(1));
  //     autonomousCommand.execute();
  //   }
  //   else if (path1.equals("Middle")) {
  //     autonomousCommand = autonomousCommand.andThen(PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryMiddleForward);
  //     autonomousCommand = autonomousCommand.andThen(() -> lawnmower.ballDump(1));
  //     autonomousCommand.execute();
  //   }
  //   else if (path1.equals("Right")) 
  //     autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryRightForward);
  //     autonomousCommand = autonomousCommand.andThen(PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryRightForward);
  //     autonomousCommand = autonomousCommand.andThen(() -> lawnmower.ballDump(1));
  //     autonomousCommand.execute();
  //   }
  //   if (path2.equals("Left")) {
  //     autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryLeftBack);
  //     autonomousCommand.execute();
  //   }
  //   else if (path2.equals("Middle")) {
  //     autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryMiddleBack);
  //     autonomousCommand.execute();
  //   }
  //   else if (path2.equals("Right")) {
  //     autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryRightBack);
  //     autonomousCommand.execute();
  //   }
  // } 
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    for (SubsystemBase subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static double getCurrentAngle() {
    return currentAngle;
  }
}
