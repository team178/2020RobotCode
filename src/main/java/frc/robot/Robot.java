/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  public static SendableChooser<String> startPath = new SendableChooser<>();
  public static SendableChooser<String> endLocation = new SendableChooser<>();

  // Declare subsystems
  public static DriveTrain drivetrain;
  public static LawnMower lawnmower;
  public static WheelOfFortuneContestant wheeloffortunecontestant;
  private static double currentAngle;
  public static LightsArduino lights; 
  public static LightStrip lightStrip;
  public static Climber climber;

  public static String gameData;
  public static double tof1Previous;
  public static double tof2Previous;

  //Declare joysticks
  public static ThrustmasterJoystick mainController;
	public static XboxController auxController;
  
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
    
    startPath.addOption("Left","Left");
    startPath.addOption("Middle","Middle");
    startPath.addOption("Right","Right");

    endLocation.addOption("Left","Left");
    endLocation.addOption("Middle","Middle");
    endLocation.addOption("Right","Right");


    SmartDashboard.putData("AutoLocation", startPath);
    SmartDashboard.putData("AutoLocation", endLocation);

    drivetrain = new DriveTrain();
    lawnmower = new LawnMower();
    wheeloffortunecontestant = new WheelOfFortuneContestant();
    climber = new Climber();

    //lights
    lights = new LightsArduino(Port.kOnboard, RobotMap.lightsI2CAddress);
    lightStrip = new LightStrip(RobotMap.lightsPWM, RobotMap.numOfLEDs);
    
    
    drivetrain.calibrateGyro();
    gameData = "";
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    
    //init joysticks
    mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
    auxController = new XboxController(RobotMap.JoystickPortXBoxAux);


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
    SmartDashboard.putNumber("Gyro Reading", drivetrain.getGyroReading());
    SmartDashboard.putNumber("Balls in Lawn Mower", lawnmower.getCounter());

    //Gyro stuff
    if(drivetrain.getGyroReading()%360 == 0)
    {
      currentAngle = drivetrain.getGyroReading();
    } else {
      currentAngle = Math.abs(drivetrain.getGyroReading()%360);
    }
    System.out.println("Gyro Reading: " + drivetrain.getGyroReading());
    System.out.println("Current Angle Reading: " + currentAngle);

    climber.periodic();
    drivetrain.periodic();
    lights.periodic();
    wheeloffortunecontestant.periodic();

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
  //  autonomousCommand = AutonomousSelector.getAutonomousCommand();
  CommandScheduler.getInstance().run();
  Command autonomousCommand;
  //Might scrw up if keep changing startPath and endLocation
  if (startPath.getSelected() != "" && endLocation.getSelected() != ""){
    String path1 = startPath.getSelected();
    String path2 = endLocation.getSelected();
    if (path1.equals("Left")) {
      autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryLeftForward)
          .andThen(() -> lawnmower.ballDump(1));
      autonomousCommand.execute();
    }
    else if (path1.equals("Middle")) {
      autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryMiddleForward)
          .andThen(() -> lawnmower.ballDump(1));
      autonomousCommand.execute();
    }
    else if (path1.equals("Right")) {
      autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryRightForward)
          .andThen(() -> lawnmower.ballDump(1));
      autonomousCommand.execute();
    }
    if (path2.equals("Left")) {
      autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryLeftBack);
      autonomousCommand.execute();
    }
    else if (path2.equals("Middle")) {
      autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryMiddleBack);
      autonomousCommand.execute();
    }
    else if (path2.equals("Right")) {
      autonomousCommand = PossibleTrajectories.getRamseteCommand(PossibleTrajectories.TrajectoryRightBack);
      autonomousCommand.execute();
    }
  } 
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
