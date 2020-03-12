/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  // public static LightsArduino lights;
  public static Climber climber;
  
  // Declare joysticks
  public static ThrustmasterJoystick mainController;
  public static XboxController auxController;
  // public static XboxController backupMainController;
  // public static boolean backupMainBeingUsed = false;
  
  // Declare Shuffleboard Dropdowns for autonomous
  public static SendableChooser<Command> startingLoc = new SendableChooser<>();
  // public static SendableChooser<Integer> delay = new SendableChooser<>();
  
  // Declare autonomous command
  private Command autonomousCommand;
  
  // USB Camera declarations
  public VideoSink camserv1;
  public VideoSink camserv2;
  public UsbCamera primary;
  public UsbCamera secondary;
  public UsbCamera climberCam;

  //Fields
  public boolean primaryTrigger;
  public int secondaryIndex;
  
  public static String gameData;
  private static double currentAngle;

  public NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  public boolean gettingValues;
  public double xValue;
  public double yValue;
  public double area;
  
  @Override
  public void robotInit() {
    pdp = new PowerDistributionPanel(RobotMap.PDP);
    driveTrain = new DriveTrain();
    lawnMower = new LawnMower();
    wheelOfFortuneContestant = new WheelOfFortuneContestant();
    climber = new Climber();

    camserv1 = CameraServer.getInstance().getServer();
    camserv2 = CameraServer.getInstance().getServer();

    primary = CameraServer.getInstance().startAutomaticCapture("intake", 1);
    primary.setFPS(14);
    primary.setPixelFormat(PixelFormat.kYUYV);

    secondary = CameraServer.getInstance().startAutomaticCapture("shooter", 0);
    secondary.setFPS(14);
    secondary.setPixelFormat(PixelFormat.kYUYV);

    climberCam = CameraServer.getInstance().startAutomaticCapture("climber", 2);
    climberCam.setFPS(14);
    climberCam.setPixelFormat(PixelFormat.kYUYV);

    primaryTrigger = true;
    secondaryIndex = 0;

    camserv1.setSource(primary);
    camserv2.setSource(secondary);

    //lights
    // lights = new LightsArduino(Port.kMXP, RobotMap.lightsI2CAddress);    
    
    driveTrain.calibrateGyro();
    driveTrain.resetEncoders();
    gameData = "";
    
    //init joysticks
    mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
    auxController = new XboxController(RobotMap.JoystickPortXBoxAux);
    // backupMainController = new XboxController(RobotMap.JoystickBackupMain);
    configButtonControls();

    startingLoc.setDefaultOption("Yeet n dump", Autos.BasicMiddleAuto);
    startingLoc.addOption("Yeet n dump", Autos.BasicMiddleAuto);
    startingLoc.addOption("Just yeet", new AutoDrive(-0.5, 5));
    /*
    startingLoc.addOption("Left", new SequentialCommandGroup(
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[0])),
      new AutoBallDump(),
      PathWeaverTrajecotires.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[3]))
    ));
    startingLoc.addOption("Middle", new SequentialCommandGroup(
      PathWeaverTrajectories.getR
      amseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[1])),
      new AutoBallDump(),
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[3]))
    ));
    startingLoc.addOption("Right", new SequentialCommandGroup(
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[2])),
      new AutoBallDump(),
      PathWeaverTrajectories.getRamseteCommand(createTrajectory(PathWeaverTrajectories.BlueTrajectories[3]))
    ));
    */
    gettingValues = false;

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
    //changePrimaryCamera();
    //allCameraChange();

    //Gyro stuff
    if(driveTrain.getGyroReading()%360 == 0) {
      currentAngle = driveTrain.getGyroReading();
    } else {
      currentAngle = Math.abs(driveTrain.getGyroReading()%360);
    }

    if (limelightTable.getEntry("tv").getDouble(0.0) == 0) {
      gettingValues = false;
    } else {
      gettingValues = true;
    }

    xValue = limelightTable.getEntry("tx").getDouble(0.0);
    yValue = limelightTable.getEntry("ty").getDouble(0.0);
    area = limelightTable.getEntry("ta").getDouble(0.0);

    // SmartDashboard.putNumber("Gyro Reading", drivetrain.getGyroReading());
    SmartDashboard.putNumber("Balls in Lawn Mower", lawnMower.getCounter());
    SmartDashboard.putBoolean("Conveyor Not Moving", lawnMower.positionOverride());
    SmartDashboard.putData("Starting Location", startingLoc);
    // // SmartDashboard.putData("Auto Delay", delay);
    SmartDashboard.putNumber("Encoder left", driveTrain.leftPosition.get());
    SmartDashboard.putNumber("Encoder right", driveTrain.rightPosition.get());
    SmartDashboard.putBoolean("Limelight Detecting Objects", gettingValues);
    SmartDashboard.putNumber("Limelight X", xValue);
    SmartDashboard.putNumber("Limelight Y", yValue);
    SmartDashboard.putNumber("Limelight Area", area);
    
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
    //mainController.rightPadTop2.whenPressed(() -> toggleBackupMainUsage());
    mainController.leftPadTop3.whenPressed(() -> clearStickyFaults());
    // backupMainController.x.whenPressed(() -> drivetrain.toggleDriveDirection());
    mainController.headLeft.whenPressed(() -> togglePrimary());
    mainController.headRight.whenPressed(() -> toggleSecondary());
    
    //Aux buttons
    auxController.a.whenPressed(() -> wheelOfFortuneContestant.spinPC(1)).whenReleased(() -> wheelOfFortuneContestant.spinPC(0));
    auxController.b.whenPressed(() -> lawnMower.ballDump(1, .6)).whenReleased(() -> lawnMower.ballDump(0, 0));
    auxController.x.whenPressed(() -> wheelOfFortuneContestant.spinRC(1)).whenReleased(() -> wheelOfFortuneContestant.spinRC(0));
    auxController.y.whenPressed(() -> lawnMower.ballDump(1, 1)).whenReleased(() -> lawnMower.ballDump(0, 0));

    auxController.back.whenPressed(() -> wheelOfFortuneContestant.extendContestant());
    auxController.start.whenPressed(() -> wheelOfFortuneContestant.retractContestant());
    Robot.auxController.leftBumper.whenPressed(() -> climber.extendHook(true));
    Robot.auxController.rightBumper.whenPressed(() -> climber.retractHook());
  }

  public void clearStickyFaults() {
    pdp.clearStickyFaults();
    System.out.println(pdp.getVoltage());
  }

  public void togglePrimary() {
    if (!primaryTrigger) {
      camserv1.setSource(secondary);
      primaryTrigger = true;
    } else if (primaryTrigger) {
      camserv1.setSource(primary);
      primaryTrigger = false;
    }
  }

  public void toggleSecondary() {
    if (secondaryIndex == 0) {
      camserv2.setSource(climberCam);
      secondaryIndex = 1;
    } else if (secondaryIndex == 1) {
      camserv2.setSource(primary);
      secondaryIndex = 2;
    } else if (secondaryIndex == 2) {
      camserv2.setSource(secondary);
      secondaryIndex = 0;
    }
  }

  // public void toggleBackupMainUsage() {
  //   backupMainBeingUsed = backupMainBeingUsed ? false : true;
  // }
}