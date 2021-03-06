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

  public WheelOfFortuneContestant() {
    contestant = new TalonSRX(RobotMap.contestant);
    deployer = new Solenoid(RobotMap.WOFdeployer);
    colorsensor = new ColorSensor();
    rot = 0;
    initColor = getColor();
    countTrigger = false;
  }

  // these are our test values, not the acutal competition values.
  public static final Color Blue = ColorMatch.makeColor(0.208, 0.471, 0.320);
  public static final Color Green = ColorMatch.makeColor(0.240, 0.568, 0.191);
  public static final Color Red = ColorMatch.makeColor(0.504, 0.353, 0.142);
  public static final Color Yellow = ColorMatch.makeColor(0.305, 0.546, 0.140);
  public static final Color Black = ColorMatch.makeColor(0,0,0);

  public void extendContestant() {
    deployer.set(true);
  }

  public void retractContestant() {
    deployer.set(false);
  }

  public void spinContestant(double speed) {
    contestant.set(ControlMode.PercentOutput, speed);
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

  public void spinRC(double speed) {
    if (!rotationControl(3)) {
      spinContestant(speed);
    } else {
      spinContestant(0);
    }
  }

  public void spinPC(double speed) {
    if (!positionControl()) {
      spinContestant(speed);
    } else {
      spinContestant(0);
    }
  }

  public void spinContestant(double speed, boolean override) {
    if (override) {
      contestant.set(ControlMode.PercentOutput, speed);
    } else {
      if(Robot.gameData.length() > 0) {
        spinPC(speed);
      } else {
        spinRC(speed);
      }
    }
  }

  // should only have to apply "spinPC" and "spinRC" to buttons/triggers

  @Override
  public void periodic() {
    if (Robot.auxController.getLeftTrigger() > 0) {
      spinContestant(Robot.auxController.getLeftTrigger(), true);
    }
  }

}

  
   

