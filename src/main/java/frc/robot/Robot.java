/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AddressableLED;//lights
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.autonomous.AutonomousSelector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LawnMower;
import frc.robot.subsystems.WheelOfFortuneContestant;

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
  public static ColorSensorV3 colorSensor;
  public static OI oi;
  public static WheelOfFortuneContestant wheeloffortunecontestant;
  private static double currentAngle;
  private static final double smallTolerance = .1;

  /* //moved to LightStrip subsystem 
  //lights
  public static DriverStation ds;
  public static AddressableLED LEDStrip1;
  public static AddressableLED LEDStrip2;
  public static AddressableLEDBuffer AddressableLEDBuffer1;
  public static AddressableLEDBuffer AddressableLEDBuffer2;
  public static int LEDCount1=10;
  public static int LEDCount2=10;
  */
  public static double tof1Previous;
  public static double tof2Previous;
  
  //Declare autonomous command
  private Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    drivetrain = new DriveTrain();
    colorSensor = new ColorSensorV3(null);
    lawnmower = new LawnMower();
    oi = new OI();
    wheeloffortunecontestant = new WheelOfFortuneContestant();
    drivetrain.calibrateGyro();
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    /* //moved to LightStrip subsystem 
    //lights
    ds = DriverStation.getInstance();
    LEDStrip1= new AddressableLED(9);
    LEDStrip2= new AddressableLED(8);
    AddressableLEDBuffer1= new AddressableLEDBuffer(LEDCount1);
    AddressableLEDBuffer2= new AddressableLEDBuffer(LEDCount2);
    LEDStrip1.setLength(AddressableLEDBuffer1.getLength());
    LEDStrip2.setLength(AddressableLEDBuffer2.getLength());
    LEDStrip1.setData(AddressableLEDBuffer1);
    LEDStrip2.setData(AddressableLEDBuffer2);
    LEDStrip1.start();
    LEDStrip2.start();
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
    drivetrain.calibrateGyro();
    lawnmower.updateTof1Distance();
    lawnmower.updateTof2Distance();
    lawnmower.updateTof3Distance();
    SmartDashboard.putNumber("Gyro Reading", drivetrain.getGyroReading());
    SmartDashboard.putNumber("TOF 1 Distance", lawnmower.getTof1Distance());
    SmartDashboard.putNumber("TOF 2 Distance", lawnmower.getTof2Distance());
    SmartDashboard.putNumber("TOF 3 Distance", lawnmower.getTof3Distance());
    SmartDashboard.putNumber("Balls in Lawn Mower", lawnmower.getCounter());
    SmartDashboard.putString("TOF 1 Edge", lawnmower.getTof1Edge());
    SmartDashboard.putString("TOF 2 Edge", lawnmower.getTof2Edge());
    SmartDashboard.putString("TOF 3 Edge", lawnmower.getTof3Edge());
    SmartDashboard.putString("Color", wheeloffortunecontestant.getColor());
    System.out.println("Gyro reading:" + drivetrain.getGyroReading());
    drivetrain.resetGyro();

    //Gyro stuff
    if(drivetrain.getGyroReading()%360 == 0)
    {
      currentAngle = drivetrain.getGyroReading();
    } else {
      currentAngle = Math.abs(drivetrain.getGyroReading()%360);
    }
    System.out.println("Gyro Reading: " + drivetrain.getGyroReading());
    System.out.println("Current Angle Reading: " + currentAngle);
    
    //lights

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

  /* //moved to LightStrip subsystem 
  //lights
  public void lightsaber(){
    boolean isRed = ds.getAlliance() != Alliance.Blue;

    if(isRed == true){
      for(var height=0; height<LEDCount1; height++){
        AddressableLEDBuffer1.setRGB(height, 99,0,0);
        //AddressableLEDBuffer1.delay(20);  FIND DELAY FUNCTION
        LEDStrip1.setData(AddressableLEDBuffer1);
      }
    }
    else if(isRed == false){
      for(var height=0; height<LEDCount1; height++){
        AddressableLEDBuffer1.setRGB(height,31,235,253);
        //FIND Delay Function
        LEDStrip1.setData(AddressableLEDBuffer1);
      }
    }
  }
  */
}
