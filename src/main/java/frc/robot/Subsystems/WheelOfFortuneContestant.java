/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import libs.IO.ColorSensor;

public class WheelOfFortuneContestant extends SubsystemBase {

  private VictorSPX contestant;
  private ColorSensor colorsensor;
  private double rot;
  private char initColor;
  private boolean countTrigger;

  public WheelOfFortuneContestant() {
    contestant = new VictorSPX(RobotMap.contestant);
    colorsensor = new ColorSensor();
    rot = 0;
    initColor = getColor();
    countTrigger = false;
  }

  public static final Color Blue = ColorMatch.makeColor(0.153, 0.445, 0.402);
  public static final Color Green = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color Red = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color Yellow = ColorMatch.makeColor(0.319, 0.545, 0.136);
  public static final Color Black = ColorMatch.makeColor(0,0,0);
  private String gameData = DriverStation.getInstance().getGameSpecificMessage();

  public char findGameDataColor() {
    if(gameData.length() > 0) {
      return gameData.charAt(0);
    }
    return 'N';
  }
 
 
  
  

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

  public boolean rotationControl(int desiredRotations) {
    if (getRotations() < desiredRotations) {
      return false;
    }
    return true;
  }

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

  
  public void periodic() {
    
  }
}

  
   

