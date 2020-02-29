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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.autonomous.AutoLeftShoot;
import frc.robot.autonomous.AutoMiddleShoot;
import frc.robot.autonomous.AutoRightShoot;
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
  
  public static SendableChooser<Boolean> match = new SendableChooser<>();
  public static SendableChooser<Command> autoModes = new SendableChooser<>();
  public static SendableChooser<Integer> preLoaded = new SendableChooser<>();
  
  //Declare autonomous command
  private RamseteCommand trajectoryCommand;
  private Command autonomousCommand;

  //USB Camera declarations
  public static CameraServer camserv;
  public static UsbCamera camera;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    drivetrain = new DriveTrain();
    lawnmower = new LawnMower();
    wheeloffortunecontestant = new WheelOfFortuneContestant();
    climber = new Climber();

    //lights
    //lights = new LightsArduino(Port.kOnboard, RobotMap.lightsI2CAddress);
    //lightStrip = new LightStrip(RobotMap.lightsPWM, RobotMap.numOfLEDs);
    
    
    drivetrain.calibrateGyro();
    gameData = "";
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    
    //init joysticks
    mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
    auxController = new XboxController(RobotMap.JoystickPortXBoxAux);
    configButtonControls();

    //Camera initializations
    camserv = CameraServer.getInstance();

    //Camera 1
    camera = camserv.startAutomaticCapture("cam1", 0);
    //camera.setResolution(160, 90);
    camera.setFPS(14);
    camera.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras
    
    //Camera 1
    camera = camserv.startAutomaticCapture("cam1", 1);
    //camera.setResolution(160, 90);
    camera.setFPS(14);
    camera.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras

    autoModes.addOption("Left", new AutoLeftShoot());
    autoModes.addOption("Middle", new AutoMiddleShoot());
    autoModes.addOption("Right", new AutoRightShoot());

    preLoaded.addOption("0", 0);
    preLoaded.addOption("1", 1);
    preLoaded.addOption("2", 2);
    preLoaded.addOption("3", 3);

    match.setDefaultOption("Yes", true);
    match.addOption("Yes", true);
    match.addOption("No", false);
  }

  private void configButtonControls() {
    //Main buttons
    mainController.trigger.whenPressed(() -> drivetrain.toggleDriveDirection());
    mainController.rightPadBottom3.whenPressed(() -> lawnmower.resetCounter());
    
    //Aux buttons
    auxController.a.whenPressed(() -> wheeloffortunecontestant.spinPC(1));
    auxController.x.whenPressed(() -> wheeloffortunecontestant.spinRC(1));
    auxController.back.whenPressed(() -> wheeloffortunecontestant.extendContestant());
    auxController.start.whenPressed(() -> wheeloffortunecontestant.retractContestant());
    auxController.leftBumper.whenPressed(() -> climber.extendHook());
    auxController.rightBumper.whenPressed(() -> climber.retractHook());//.whileHeld(() -> climber.windWinch(0.4))
      //.whenReleased(() -> climber.windWinch(0));
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

    //camera stuffb 
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
    //Gyro stuff
    if(drivetrain.getGyroReading()%360 == 0)
    {
      currentAngle = drivetrain.getGyroReading();
    } else {
      currentAngle = Math.abs(drivetrain.getGyroReading()%360);
    }

  /*  climber.periodic();
    drivetrain.periodic();
    //lights.periodic();
    lawnmower.periodic();
    wheeloffortunecontestant.periodic(); */

    SmartDashboard.putNumber("Gyro Reading", drivetrain.getGyroReading());
    SmartDashboard.putNumber("Balls in Lawn Mower", lawnmower.getCounter());
    SmartDashboard.putNumber("TOF 1 Reading", lawnmower.getTof1Distance());
    SmartDashboard.putNumber("TOF 2 Reading", lawnmower.getTof2Distance());
    SmartDashboard.putNumber("TOF 3 Reading", lawnmower.getTof3Distance());
    SmartDashboard.putString("TOF 1 Edge", lawnmower.getTof1Edge());
    SmartDashboard.putString("TOF 2 Edge", lawnmower.getTof2Edge());
    SmartDashboard.putString("TOF 3 Edge", lawnmower.getTof3Edge());
    SmartDashboard.putBoolean("Conveyor Not Moving", lawnmower.positionOverride());
    SmartDashboard.putData("Auto Chooser", autoModes);
    SmartDashboard.putData("Balls Pre-Loaded", preLoaded);
    SmartDashboard.putData("Match?", match);

    if (match.getSelected() == false)
      auxController.b.whenPressed(() -> climber.windWinch(-0.1)).whenReleased(() -> climber.windWinch(0));

    CommandScheduler.getInstance().run();
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
    lawnmower.counter = preLoaded.getSelected();
    autonomousCommand = autoModes.getSelected();
  
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
