package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final double BLUE = 0.4;
  private final double RED = 0.4;
  private final double GREEN = 0.4;
  private final double YELLOWDISTINCT = 0.25;

  private final String colorBlue = "Blue";
  private final String colorGreen = "Green";
  private final String colorRed = "Red";
  private final String colorYellow = "Yellow";
  private final String noColor = "No Color detected";

  private int rot = 0;
  private int subRot = 0;

  private final Color detectedColor = m_colorSensor.getColor();

  @Override
  public void robotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    //    final Color detectedColor = m_colorSensor.getColor();
    //public static final ColorMatcher m_colorSensor = new ColorMatch();

    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    final double IR = m_colorSensor.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR); 

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

  }

  public String getColor()
  {
    if(detectedColor.blue >= BLUE)
    {
      return colorBlue;
    } else if(detectedColor.green >= GREEN)
    {
      return colorGreen;
    } else if(detectedColor.red >= RED)
    {
      return colorRed;
    } else if(detectedColor.green >= GREEN && detectedColor.red >= YELLOWDISTINCT) //conditioning for yellow color
    {
      return colorYellow;
    } else
    {
      return noColor;
    }
  }

  public void wheelRotations()
  {
    if(detectedColor.blue == BLUE)
    {
      rot++;
    }

    if(rot%2 == 0)
    {
      subRot += rot/2;
      rot = 0;
    }
  }
}
  
