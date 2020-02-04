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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import libs.IO.ColorSensor;

public class WheelOfFortuneContestant extends SubsystemBase {

  private VictorSPX contestant;
  private ColorSensor colorsensor;

  public WheelOfFortuneContestant() {
    contestant = new VictorSPX(RobotMap.contestant);
    colorsensor = new ColorSensor();
  }

  public static final Color Blue = ColorMatch.makeColor(0.136, 0.412, 0.450);
  public static final Color Green = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color Red = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color Yellow = ColorMatch.makeColor(0.293, 0.561, 0.144);
  public static final Color Black = ColorMatch.makeColor(0,0,0);
  


  public void spinToWin(double power) {
    contestant.set(ControlMode.PercentOutput, power);
  }
    
  public String getColor() {
    Color c = colorsensor.detectColor();
    if (compareColors(c, Blue))
      return "Blue";
    if (compareColors(c, Green))
      return "Green";
    if (compareColors(c, Red))
      return "Red";
    if (compareColors(c, Yellow))
      return "Yellow";
    return "No Color";
  }

  public boolean compareColors(Color a, Color b) {
    if ((a.red < b.red + 0.02) && (a.red > b.red - 0.02)) {
      if ((a.green < b.green + 0.02) && (a.green > b.green - 0.02)) {
        if ((a.blue < b.blue + 0.02) && (a.blue > b.blue - 0.02)) {
          return true;
        }
      } 
    }
    return false;
  }
    
    

  
  public void periodic() {
    SmartDashboard.putString("Color", getColor());
  }
}
   

