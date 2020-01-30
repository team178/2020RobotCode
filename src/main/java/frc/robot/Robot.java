/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LawnMower;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project....
 */
public class Robot extends TimedRobot {

  //Declare subsystems
  public static DriveTrain drivetrain;
  public static LawnMower lawnmower;
  public static ColorSensor colorSensor;
  public static OI oi;
  public static AddressableLED LEDStrip1;
  public static AddressableLED LEDStrip2;
  public static AddressableLEDBuffer AddressableLEDBuffer1;
  public static AddressableLEDBuffer AddressableLEDBuffer2;
  public static int LEDCount1=10;
  public static int LEDCount2=10;
  
  //Declare autonomous command
  private Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  @Override
  public void robotInit() {
    drivetrain = new DriveTrain();
    colorSensor = new ColorSensor();
    lawnmower = new LawnMower();
    oi = new OI();
    drivetrain.calibrateGyro();
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    //lights
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
    SmartDashboard.putNumber("Gyro Reading", drivetrain.getGyroReading());
    SmartDashboard.putNumber("TOF 1 Reading", lawnmower.getTof1Distance());
    SmartDashboard.putNumber("TOF 2 Reading", lawnmower.getTof2Distance());
//    SmartDashboard.putNumber("TOF 3 Reading", lawnmower.getTof3Distnace());
    System.out.println("Gyro reading:" + drivetrain.getGyroReading());
    drivetrain.resetGyro();



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
    autonomousCommand = AutonomousSelector.getAutonomousCommand();
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


  //lights
  public void lightsaber(){
    boolean isRed=false;
    if(isRed==true){
      for(var height=0; height<LEDCount1; height++){
        AddressableLEDBuffer1.setRGB(height, 99,0,0);
        //AddressableLEDBuffer1.delay(20);  FIND DELAY FUNCTION
        LEDStrip1.setData(AddressableLEDBuffer1);
      }
    }
    if(isRed==false){
      for(var height=0; height<LEDCount1; height++){
        AddressableLEDBuffer1.setRGB(height,31,235,253);
        //FIND Delay Function
        LEDStrip1.setData(AddressableLEDBuffer1);
      }

    }
  }
}
