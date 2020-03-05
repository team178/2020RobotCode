/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
import frc.robot.commands.AutoBallDump;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.MoveToAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LawnMower;
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

  //Declare PDP
  public static PowerDistributionPanel pdp;

  // Declare subsystems
  public static DriveTrain drivetrain;
  public static LawnMower lawnmower;
  public static WheelOfFortuneContestant wheeloffortunecontestant;
  private static double currentAngle;
  public static LightsArduino lights;
  public static Climber climber;

  // FMS Game Data for Position Control
  public static String gameData;

  // Declare joysticks
  public static ThrustmasterJoystick mainController;
  public static XboxController auxController;

  // Declare Shuffleboard Dropdowns for autonomous
  public static SendableChooser<Command> startingLoc = new SendableChooser<>();
  public static SendableChooser<Integer> preLoaded = new SendableChooser<>();
  // public static SendableChooser<String> alliance = new SendableChooser<>();

  // Declare autonomous command
  private Command autonomousCommand;

  // USB Camera declarations
  public UsbCamera intake = CameraServer.getInstance().startAutomaticCapture("intake", 0);
  public UsbCamera shooter = CameraServer.getInstance().startAutomaticCapture("shooter", 1);
  public UsbCamera climberCam = CameraServer.getInstance().startAutomaticCapture("climber", 2);
  public UsbCamera color = CameraServer.getInstance().startAutomaticCapture("color", 3);

  public MjpegServer primary = CameraServer.getInstance().addSwitchedCamera("primary");
  public MjpegServer secondary = CameraServer.getInstance().addSwitchedCamera("secondary");

  public boolean primaryTrigger = false;
  public int secondaryTrigger = 1;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    pdp = new PowerDistributionPanel(RobotMap.PDP);
    drivetrain = new DriveTrain();
    lawnmower = new LawnMower();
    wheeloffortunecontestant = new WheelOfFortuneContestant();
    climber = new Climber();

//    camShooter = new UsbCamera("shooter", 1);
//    camIntake = new UsbCamera("intake", 0);
//    camColorSensor = new UsbCamera("color", 3);
//  camClimber = new UsbCamera("climber", 2);
    primary.setSource(intake);
    secondary.setSource(shooter);

    //lights
    lights = new LightsArduino(Port.kMXP, RobotMap.lightsI2CAddress);    
    
    drivetrain.calibrateGyro();
    drivetrain.resetEncoders();
    gameData = "";
    
    //init joysticks
    mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
    auxController = new XboxController(RobotMap.JoystickPortXBoxAux);
    configButtonControls();

    //Camera initializations


  //  camShooter.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  //  camIntake.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    startingLoc.addOption("Side", new BasicLeftAuto());
    startingLoc.addOption("Middle", new BasicMiddleAuto());
    startingLoc.addOption("Opposite", new AutoDrive(-0.5, 5));
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
    preLoaded.setDefaultOption("0", 0);
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
    //changePrimaryCamera();
    //allCameraChange();

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

    lawnmower.counter = preLoaded.getSelected();

    // if(camSecondaryCounter == 0){
    //   getCamIntake();
    // }

    // if(camSecondaryCounter == 1){
    //   getCamShooter();
    // }

    // if(camSecondaryCounter == 2){
    //   getCamClimber();
    // }

    // if(camSecondaryCounter == 3){
    //   getCamColorSensor();
    // }
    
    CommandScheduler.getInstance().run();
    lights.periodic();
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
    drivetrain.resetEncoders();
    drivetrain.resetGyro();
    
    autonomousCommand = startingLoc.getSelected();
    lawnmower.counter = preLoaded.getSelected();
  
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
    mainController.rightPadBottom3.whenPressed(() -> drivetrain.toggleDriveDirection());
//    mainController.rightPadBottom2.whenPressed(() -> lawnmower.resetCounter());
    mainController.rightPadTop3.whenPressed(() -> drivetrain.resetEncoders());
    mainController.headLeft.whenPressed(() -> switchPrimaryCam());
    mainController.headRight.whenPressed(() -> switchSecondaryCam());
      mainController.leftPadTop1.whenPressed(() -> clearStickyFaults());
    
    //Aux buttons
    auxController.a.whenPressed(() -> wheeloffortunecontestant.spinPC(1)).whenReleased(() -> wheeloffortunecontestant.spinPC(0));
    auxController.b.whenPressed(() -> lawnmower.moveConveyor(-0.2)).whenReleased(() -> lawnmower.moveConveyor(0));
    auxController.x.whenPressed(() -> wheeloffortunecontestant.spinRC(1)).whenReleased(() -> wheeloffortunecontestant.spinRC(0));
    auxController.y.whenPressed(() -> lawnmower.ballDump(0.6)).whenReleased(() -> lawnmower.ballDump(0));
    auxController.back.whenPressed(() -> wheeloffortunecontestant.extendContestant());
//    auxController.back.whenPressed(() -> changeSecondaryCamera(4));
    auxController.start.whenPressed(() -> wheeloffortunecontestant.retractContestant());
    Robot.auxController.leftBumper.whenPressed(() -> climber.extendHook());
//    Robot.auxController.leftBumper.whenPressed(() -> changeSecondaryCamera(3));
    Robot.auxController.rightBumper.whenPressed(() -> climber.retractHook());
  }
  /*
  
  public void getPrimaryCams(){
    private int PrimeCounter = 0;
    if(maincontroller.leftHead.get()){
      PrimeCounter ++;
    }
    else{
    PrimeCounter = 0;}

    if(PrimeCounter == 0){

    }

  }
  
  */
  public void switchPrimaryCam() {
    if (!primaryTrigger) {
      primary.setSource(shooter);
      primaryTrigger = true;
    } else {
      primary.setSource(intake);
      primaryTrigger = false;
    }
  }

  public void switchSecondaryCam() {
    if (secondaryTrigger == 0) {
      secondary.setSource(shooter);
      secondaryTrigger = 1;
    } else if (secondaryTrigger == 1) {
      secondary.setSource(climberCam);
      secondaryTrigger = 2;
    } else if (secondaryTrigger == 2) {
      secondary.setSource(color);
      secondaryTrigger = 3;
    } else if (secondaryTrigger == 3) {
      secondary.setSource(intake);
      secondaryTrigger = 0;
    }
  }

  public void clearStickyFaults() {
    pdp.clearStickyFaults();
    System.out.println(pdp.getVoltage());
  }
}