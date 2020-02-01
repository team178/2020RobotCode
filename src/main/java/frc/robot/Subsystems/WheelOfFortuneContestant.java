/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ColorSensor;

public class WheelOfFortuneContestant extends SubsystemBase {
  /**
   * Creates a new WheelOfFortuneContestant.
   */
  public WheelOfFortuneContestant() {
    
  }


  private final I2C.Port i2cPort = I2C.Port.kOnboard;



  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  Color detectedColor = m_colorSensor.getColor();

  private int rot = 0;

  private int subRot = 0;
 
  private final static ColorMatch m_colorMatcher = new ColorMatch();
  // private static final Color BLUE = new ColorMatch.makeColor(0, 0, );
  public static final Color Blue = ColorMatch.makeColor(0.136, 0.412, 0.450);
  public static final Color Green = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color Red = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color Yellow = ColorMatch.makeColor(0.293, 0.561, 0.144);
  private final double PROXIMITY = 0; // IR
  // this isn't the final value, this will change depending on sensor placement
  private final static Color DESIRE = Blue;
 

    /**
     * Run the color match algorithm on our detected color
     */
    
    
    
    

  
  public void periodic() {
    // This method will be called once per scheduler run
    ColorSensor s1 = new ColorSensor();

    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    final double IR = m_colorSensor.getIR();
    
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    
    

    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    final int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
    //Proximity can be used to make sure consistant distances are being used when imputing blue values
    
    
  }
  public int getRotations() {
   
    return rot;
  }
  ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

  
  public void updateRotations(){
  
    
    


    if(match.color == Blue) 
    {
      
    }

    if(subRot%2 == 0)
    {
      subRot /= 2;
      rot += subRot;
      subRot = 0;
      //takes amount of time BLUE pops up, and creates a counter. Counter is then converted into revolutions of the wheel
      //rot is the rotation value
    }
  }
    public static void ColorMatcher() {
      m_colorMatcher.addColorMatch(Blue);
      m_colorMatcher.addColorMatch(Green);
      m_colorMatcher.addColorMatch(Red);
      m_colorMatcher.addColorMatch(Yellow);
    }
    
    public void desiredColor()//ask vivek to put javadoc here
    {
      if(match.color == DESIRE){//Write this when you have the controller panal motor code
        
        
      }
    


    }
    

     


    
}
   

