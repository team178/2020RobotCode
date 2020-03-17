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
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import libs.IO.*;
import libs.limelight.LimelightCamera;

public class Robot extends TimedRobot {

  //Declare PDP & subsystems
  public static PowerDistributionPanel pdp;
  public static DriveTrain driveTrain;
  public static LawnMower lawnMower;
  public static WheelOfFortuneContestant wheelOfFortuneContestant;
  public static Climber climber;
  public static LimelightCamera limelight;
  
  //Declare joysticks
  public static ThrustmasterJoystick mainController;
  public static XboxController auxController;
  public static WiiRemote wiiRemote;
  
  //Declare autonomous members
  public static SendableChooser<Command> startingLoc = new SendableChooser<>();
  private Command autonomousCommand;
  
  //USB Camera declarations
  public MjpegServer camserv1;
  public MjpegServer camserv2;
  public UsbCamera primary;
  public UsbCamera secondary;
  public UsbCamera climberCam;
  public UsbCamera[] cams = {primary, secondary, climberCam};
  public int cameraIndex = 1;
  
  //Misc
  public static String gameData;
  
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

    camserv1.setSource(primary);
    camserv2.setSource(secondary);
    
    driveTrain.calibrateGyro();
    driveTrain.resetEncoders();
    gameData = "";
    
    //init joysticks
    mainController = new ThrustmasterJoystick(RobotMap.ActualJoystick);
    auxController = new XboxController(RobotMap.JoystickPortXBoxAux);
    wiiRemote = new WiiRemote(RobotMap.WiiRemote, false); //no nunchuck for now
    configButtonControls();

    startingLoc.setDefaultOption("Yeet n dump", Autos.BasicMiddleAuto);
    startingLoc.addOption("Yeet n dump", Autos.BasicMiddleAuto);
    startingLoc.addOption("Just yeet", new AutoDrive(-0.5, 5));
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    
    //General widgets
    SmartDashboard.putNumber("Balls in Lawn Mower", lawnMower.getCounter());
    SmartDashboard.putBoolean("Conveyor Not Moving", lawnMower.positionOverride());
    SmartDashboard.putData("Starting Location", startingLoc);
    SmartDashboard.putNumber("Encoder left", driveTrain.leftPosition.get());
    SmartDashboard.putNumber("Encoder right", driveTrain.rightPosition.get());
    
    //Limelight widgets
    SmartDashboard.putBoolean("Limelight Detecting Objects", limelight.isTargetFound());
    SmartDashboard.putNumber("tx", limelight.getHorizontalDegToTarget());
    SmartDashboard.putNumber("ty", limelight.getVerticalDegToTarget());
    SmartDashboard.putNumber("area", limelight.getTargetArea());
  }

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
    auxController.leftBumper.whenPressed(() -> climber.extendHook(true));
    auxController.rightBumper.whenPressed(() -> climber.retractHook());
  }

  public void clearStickyFaults() {
    pdp.clearStickyFaults();
  }

  public void toggleCameraStream() {
    cameraIndex = cameraIndex == cams.length ? 0 : cameraIndex + 1;
    camserv2.setSource(cams[cameraIndex]);
  }
}
