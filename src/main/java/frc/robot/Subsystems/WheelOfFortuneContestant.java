/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import libs.IO.ColorSensor;

public class WheelOfFortuneContestant extends SubsystemBase {

  private TalonSRX contestant;
  private Solenoid deployer;
  private ColorSensor colorsensor;
  private double rot;
  private char initColor;
  private boolean countTrigger;
  private boolean solenoidTrigger;

  public WheelOfFortuneContestant() {
    contestant = new TalonSRX(RobotMap.contestant);
    deployer = new Solenoid(RobotMap.WOFdeployer);
    colorsensor = new ColorSensor();
    rot = 0;
    initColor = getColor();
    countTrigger = false;
    solenoidTrigger = false;
  }

  public static final Color Blue = ColorMatch.makeColor(0.153, 0.445, 0.402);
  public static final Color Green = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color Red = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color Yellow = ColorMatch.makeColor(0.319, 0.545, 0.136);
  public static final Color Black = ColorMatch.makeColor(0,0,0);

  public void extendContestant() {
    deployer.set(true);
  }

  public void retractContestant() {
    deployer.set(false);
  }
  
  
  /** 
   * @return char
   * This command takes the game data color given and gives us the game data in the for of a char
   * This makes the value easier and more consistant to react with and manipulate in the code
   */
  public char findGameDataColor() {
    String gameData = Robot.gameData;
    if(gameData.length() > 0) {
      return gameData.charAt(0);
    }
    return 'N';
  }
 
  
  /** 
   * @return char
   * This takes the color values that are being gotten from the color sensor and converting them to chars
   * This allows us to create chars from the values we get, so that when we compare colors
   * we are comparing chars to chars and not strings to chars
   */
  public char getColor() {
    Color c = colorsensor.detectColor();
    if (compareColors(c, Blue)) {
      return 'B';
    }
    if (compareColors(c, Green)) {
      return 'G';
    }
    if (compareColors(c, Red)) {
      return 'R';
    }
    if (compareColors(c, Yellow)) {
      return 'Y';
    }

    return 'N';
  }

  
  /** 
   * @param a
   * @param b
   * @return boolean
   * This is the method that 
   */
  public boolean compareColors(Color a, Color b) {
    if ((a.red < b.red + 0.045) && (a.red > b.red - 0.045)) {
      if ((a.green < b.green + 0.045) && (a.green > b.green - 0.045)) {
        if ((a.blue < b.blue + 0.045) && (a.blue > b.blue - 0.045)) {
          return true;
        }
      } 
    }
    return false;
  }
    
  
  /** 
   * @return double
   */
  public double getRotations() {
    if (initColor == 'N') {
      initColor = getColor();
      return 0;
    }

    if (initColor != getColor() && getColor() != 'N') {
      countTrigger = true;
    }

    if (countTrigger) {
      if (initColor == getColor()) {
        rot += 0.5;
        countTrigger = false;
      }

      if (rot >= 5) {
        rot = 0;
      }
    }
    
    return rot;
  }

  
  /** 
   * @param desiredRotations
   * @return boolean
   */
  public boolean rotationControl(int desiredRotations) {
    if (getRotations() < desiredRotations) {
      return false;
    }
    return true;
  }

  
  /** 
   * @return boolean
   */
  public boolean positionControl() {
    if (findGameDataColor() != getColor() || findGameDataColor() == 'N') {
      return false;
    }
    return true;
  }

  public void spinRC() {
    if (!rotationControl(3)) {
      contestant.set(ControlMode.PercentOutput, 1);
    } else {
      contestant.set(ControlMode.PercentOutput, 0);
    }
  }

  public void spinPC() {
    if (!positionControl()) {
      contestant.set(ControlMode.PercentOutput, 1);
    } else {
      contestant.set(ControlMode.PercentOutput, 0);
    }
  }

  // should only have to apply "spinPC" and "spinRC" to buttons/triggers

  @Override
  public void periodic() {
    if (!Robot.auxController.rightBumper.get()) {
      solenoidTrigger = true;
    }
    
    if (Robot.auxController.rightBumper.get() && solenoidTrigger) {
      if (deployer.get()) {
        retractContestant();
      } else {
        extendContestant();
      }
      solenoidTrigger = false;
    }

    if (Robot.auxController.a.get()) {
      spinRC();
    }
    
    if (Robot.auxController.x.get()) {
      spinPC();
    }
  }

}

  
   

