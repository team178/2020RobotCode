/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.WheelOfFortuneContestant;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class SpinWheelOfFortuneToColor extends CommandBase {
  WheelOfFortuneContestant wheelOfFortuneContestant = Robot.wheeloffortunecontestant;
  
  private final static ColorMatch m_colorMatcher = new ColorMatch();
  // private static final Color BLUE = new ColorMatch.makeColor(0, 0, );
  public static final Color Blue = ColorMatch.makeColor(0.136, 0.412, 0.450);
  public static final Color Green = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color Red = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color Yellow = ColorMatch.makeColor(0.293, 0.561, 0.144);
  public static final Color Black = ColorMatch.makeColor(0,0,0);
  public static final double spinPower = 1;

  private String gameData = DriverStation.getInstance().getGameSpecificMessage();
  
  public SpinWheelOfFortuneToColor() {
    addRequirements(Robot.wheeloffortunecontestant);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          break;
        case 'G' :
          //Green case code
          break;
        case 'R' :
          //Red case code
          break;
        case 'Y' :
          //Yellow case code
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      
    }

  }

  public Color findGameDataColor()
  {
    if(gameData.length() > 0)
    {
      if(gameData.charAt(0) == 'B')
      {
        Color gameDataColor = Blue;
        return Blue;
      }

      if(gameData.charAt(0) == 'G')
      {
        Color gameDataColor = Green;
        return Green;
      }

      if(gameData.charAt(0) == 'R')
      {
        Color gameDataColor = Red;
        return Red;
      }

      if(gameData.charAt(0) == 'Y')
      {
        Color gameDataColor = Yellow;
        return Yellow;
      }
    }
    return Black;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(findGameDataColor() == wheelOfFortuneContestant.getBlueColor())
    {
      while (wheelOfFortuneContestant.getColorMatch() != wheelOfFortuneContestant.getBlueColor()){//public static VictorSPX contestant = new VictorSPX(RobotMap.contestant);
        wheelOfFortuneContestant.spinToWin(spinPower);
    } 
    }

    else if (findGameDataColor()== wheelOfFortuneContestant.getGreenColor()){

     while (wheelOfFortuneContestant.getColorMatch() != wheelOfFortuneContestant.getGreenColor()){
       wheelOfFortuneContestant.spinToWin(spinPower);
     }
      
    }
    else if(findGameDataColor()== wheelOfFortuneContestant.getRedColor()){

      while(wheelOfFortuneContestant.getColorMatch() != wheelOfFortuneContestant.getRedColor()){

        wheelOfFortuneContestant.spinToWin(spinPower);

      }
    }
    else if(findGameDataColor()== wheelOfFortuneContestant.getYellowColor()){

      while(wheelOfFortuneContestant.getColorMatch()!= wheelOfFortuneContestant.getYellowColor()){

        wheelOfFortuneContestant.spinToWin(spinPower);
      }

      }
      else{
        wheelOfFortuneContestant.spinToWin(0);
      }
    
    






  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
