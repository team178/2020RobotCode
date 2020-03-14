/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.*;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LawnMower;
import frc.robot.subsystems.WheelOfFortuneContestant;
import libs.IO.ThrustmasterJoystick;
import libs.IO.XboxController;
import libs.limelight.LimelightCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project....
 */
public class Robot extends TimedRobot {

  //Declare PDP & subsystems
  public static PowerDistributionPanel pdp;
  public static DriveTrain driveTrain;
  public static LawnMower lawnMower;
  public static WheelOfFortuneContestant wheelOfFortuneContestant;
  public static Climber climber;
  public static LimelightCamera limelight;
  
  // Declare joysticks
  public static ThrustmasterJoystick mainController;
  public static XboxController auxController;
  
  //Declare autonomous members
  public static SendableChooser<Command> startingLoc = new SendableChooser<>();
  private Command autonomousCommand;
  
  // USB Camera declarations
  public MjpegServer camserv1;
  public MjpegServer camserv2;
  public UsbCamera primary;
  public UsbCamera secondary;
  public UsbCamera climberCam;

  //Fields
  public int cameraIndex;
  
  public static String gameData;
  private static double currentAngle;
  
  @Override
  public void robotInit() {
    pdp = new PowerDistributionPanel(RobotMap.PDP);
    driveTrain = new DriveTrain();
    lawnMower = new LawnMower();
    wheelOfFortuneContestant = new WheelOfFortuneContestant();
    climber = new Climber();
    limelight = new LimelightCamera();

    camserv1 = CameraServer.getInstance().addSwitchedCamera("primary");
    camserv2 = CameraServer.getInstance().addSwitchedCamera("secondary");

    primary = CameraServer.getInstance().startAutomaticCapture("intake", 1);
    primary.setFPS(14);
    primary.setPixelFormat(PixelFormat.kYUYV);

    secondary = CameraServer.getInstance().startAutomaticCapture("shooter", 0);
    secondary.setFPS(14);
    secondary.setPixelFormat(PixelFormat.kYUYV);

    climberCam = CameraServer.getInstance().startAutomaticCapture("climber", 2);
    climberCam.setFPS(14);
    climberCam.setPixelFormat(PixelFormat.kYUYV);
    
    cameraIndex = 0;

    camserv1.setSource(primary);
    camserv2.setSource(secondary);
    
    driveTrain.calibrateGyro();
    driveTrain.resetEncoders();
    gameData = "";
    
    //init joysticks
    mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
    auxController = new XboxController(RobotMap.JoystickPortXBoxAux);
    configButtonControls();

    startingLoc.setDefaultOption("Yeet n dump", Autos.BasicMiddleAuto);
    startingLoc.addOption("Yeet n dump", Autos.BasicMiddleAuto);
    startingLoc.addOption("Just yeet", new AutoDrive(-0.5, 5));
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

    //Gyro stuff
    if(driveTrain.getGyroReading()%360 == 0) {
      currentAngle = driveTrain.getGyroReading();
    } else {
      currentAngle = Math.abs(driveTrain.getGyroReading()%360);
    }

    SmartDashboard.putNumber("Balls in Lawn Mower", lawnMower.getCounter());
    SmartDashboard.putBoolean("Conveyor Not Moving", lawnMower.positionOverride());
    SmartDashboard.putData("Starting Location", startingLoc);
    SmartDashboard.putNumber("Encoder left", driveTrain.leftPosition.get());
    SmartDashboard.putNumber("Encoder right", driveTrain.rightPosition.get());
    SmartDashboard.putBoolean("Limelight Detecting Objects", limelight.isTargetFound());
    SmartDashboard.putNumber("tx", limelight.getHorizontalDegToTarget());
    SmartDashboard.putNumber("ty", limelight.getVerticalDegToTarget());
    SmartDashboard.putNumber("area", limelight.getTargetArea());
    
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
    driveTrain.resetEncoders();
    driveTrain.resetGyro();
    
    autonomousCommand = startingLoc.getSelected();
    lawnMower.counter = 3;
  
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

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

  @Override
  public void teleopPeriodic() {

    CommandScheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }

  public static double getCurrentAngle() {
    return currentAngle;
  }

  private void configButtonControls() {
    //Main buttons
    mainController.leftPadBottom3.whenPressed(() -> driveTrain.toggleDriveDirection());
    mainController.leftPadTop3.whenPressed(() -> clearStickyFaults());
    mainController.headBottom.whenPressed(() -> toggleCameraStream());
    
    //Aux buttons
    auxController.a.whenPressed(() -> wheelOfFortuneContestant.spinPC(1)).whenReleased(() -> wheelOfFortuneContestant.spinPC(0));
    auxController.x.whenPressed(() -> wheelOfFortuneContestant.spinRC(1)).whenReleased(() -> wheelOfFortuneContestant.spinRC(0));
    auxController.y.whenPressed(() -> lawnMower.ballDump(0.7, 1)).whenReleased(() -> lawnMower.ballDump(0, 0));

    auxController.back.whenPressed(() -> wheelOfFortuneContestant.extendContestant());
    auxController.start.whenPressed(() -> wheelOfFortuneContestant.retractContestant());
    Robot.auxController.leftBumper.whenPressed(() -> climber.extendHook(true));
    Robot.auxController.rightBumper.whenPressed(() -> climber.retractHook());
  }

  public void clearStickyFaults() {
    pdp.clearStickyFaults();
    System.out.println(pdp.getVoltage());
  }

  public void toggleCameraStream() {
    if (cameraIndex == 0) {
      camserv2.setSource(climberCam);
      cameraIndex = 1;
    } else if (cameraIndex == 1) {
      camserv2.setSource(primary);
      cameraIndex = 2;
    } else if (cameraIndex == 2) {
      camserv2.setSource(secondary);
      cameraIndex = 0;
    }
  }
}