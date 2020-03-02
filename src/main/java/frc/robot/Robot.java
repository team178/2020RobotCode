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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.autonomous.BasicLeftAuto;
import frc.robot.autonomous.BasicMiddleAuto;
import frc.robot.autonomous.BasicRightAuto;
import frc.robot.autonomous.PathWeaverTrajectories;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LawnMower;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.WheelOfFortuneContestant;
import frc.robot.subsystems.LightsArduino;
import java.util.ArrayList;
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

  //FMS Game Data for Position Control
  public static String gameData;

  //Declare joysticks
  public static ThrustmasterJoystick mainController;
  public static XboxController auxController;
  
  //Declare Shuffleboard Dropdowns for autonomous
  public static SendableChooser<Command> startingLoc = new SendableChooser<>();
  public static SendableChooser<Integer> preLoaded = new SendableChooser<>();
//public static SendableChooser<String> alliance = new SendableChooser<>();
  
  //Declare autonomous command
  private Command autonomousCommand;

  //USB Camera declarations
  public static CameraServer camserv;
  public static UsbCamera camIntake;
  public static UsbCamera camShooter;
  public static UsbCamera camClimber;
  public static UsbCamera camColorWheel;
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
    drivetrain.resetEncoders();
    gameData = "";
    
    //init joysticks
    mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
    auxController = new XboxController(RobotMap.JoystickPortXBoxAux);
    configButtonControls();

    //Camera initializations
    camserv = CameraServer.getInstance();

   

    startingLoc.addOption("Left", new BasicLeftAuto());
    startingLoc.addOption("Middle", new BasicMiddleAuto());
    startingLoc.addOption("Right", new BasicRightAuto());
    startingLoc.addOption("Opposite", new AutoDrive(-0.5, 1.5));
    
    /*
    startingLoc.addOption("Left", new SequentialCommandGroup(
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[0])),
      new AutoBallDump(),
      PathWeaverTrajecotires.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[3]))
    ));
    startingLoc.addOption("Middle", new SequentialCommandGroup(
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[1])),
      new AutoBallDump(),
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[3]))
    ));
    startingLoc.addOption("Right", new SequentialCommandGroup(
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[2])),
      new AutoBallDump(),
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[3]))
    ));
    */
    
    preLoaded.addOption("0", 0);
    preLoaded.addOption("1", 1);
    preLoaded.addOption("2", 2);
    preLoaded.addOption("3", 3);
    
    /*
    alliance.addOption("Blue", "Blue");
    alliance.addOption("Red", "Red");
    */
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
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    changeCamera();

    //Gyro stuff
    if(drivetrain.getGyroReading()%360 == 0)
    {
      currentAngle = drivetrain.getGyroReading();
    } else {
      currentAngle = Math.abs(drivetrain.getGyroReading()%360);
    }

    SmartDashboard.putNumber("Gyro Reading", drivetrain.getGyroReading());
    SmartDashboard.putNumber("Balls in Lawn Mower", lawnmower.getCounter());
    SmartDashboard.putBoolean("Conveyor Not Moving", lawnmower.positionOverride());
    SmartDashboard.putData("Starting Location", startingLoc);
    SmartDashboard.putData("Balls Pre-Loaded", preLoaded);
    SmartDashboard.putNumber("Encoder left", drivetrain.leftPosition.get());
    SmartDashboard.putNumber("Encoder right", drivetrain.rightPosition.get());

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
    
    autonomousCommand = startingLoc.getSelected();
  
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

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
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

  private void configButtonControls() {
    //Main buttons
    mainController.trigger.whenPressed(() -> drivetrain.toggleDriveDirection());
    mainController.rightPadBottom3.whenPressed(() -> lawnmower.resetCounter());
    
    //Aux buttons
    auxController.a.whenPressed(() -> wheeloffortunecontestant.spinPC(1)).whenReleased(() -> wheeloffortunecontestant.spinPC(0));
    auxController.b.whenPressed(() -> lawnmower.moveConveyor(-0.2)).whenReleased(() -> lawnmower.moveConveyor(0));
    auxController.x.whenPressed(() -> wheeloffortunecontestant.spinRC(1)).whenReleased(() -> wheeloffortunecontestant.spinRC(0));
    auxController.y.whenPressed(() -> lawnmower.ballDump(0.6)).whenReleased(() -> lawnmower.ballDump(0));
    auxController.back.whenPressed(() -> wheeloffortunecontestant.extendContestant());
    auxController.start.whenPressed(() -> wheeloffortunecontestant.retractContestant());
    Robot.auxController.leftBumper.whenPressed(() -> climber.extendHook());
    Robot.auxController.rightBumper.whenPressed(() -> climber.retractHook());
  }

  public void changeCamera()
  {
    int camCounter = 0;
    if(mainController.leftPadBottom3.get()){
      if(camCounter == 0)
        camIntake = camserv.startAutomaticCapture("cam1", 0);
        //camera.setResolution(160, 90);
        camIntake.setFPS(14);
        camIntake.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras
        camCounter++;
      } 
      if(camCounter == 1){
        camShooter = camserv.startAutomaticCapture("cam2", 2);
        //camera.setResolution(160, 90);
        camShooter.setFPS(14);
        camShooter.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras
        camCounter++;
      } 
      if(camCounter == 2){
        camClimber = camserv.startAutomaticCapture("cam3", 3);
        //camera.setResolution(160, 90);
        camClimber.setFPS(14);
        camClimber.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras
        camCounter++;
      }
      if(camCounter == 3){
        camColorWheel = camserv.startAutomaticCapture("cam4", 4);
        //camera.setResolution(160, 90);
        camColorWheel.setFPS(14);
        camColorWheel.setPixelFormat(PixelFormat.kYUYV); //formats video specifications for cameras
        camCounter = 0;
      }
  }
}
