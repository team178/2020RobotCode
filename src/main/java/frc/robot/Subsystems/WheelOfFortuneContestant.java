/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WheelOfFortuneContestant extends SubsystemBase {


  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private int rot = 0;
  private int subRot = 0;
  private final double BLUE = 0.4; // value depends on lighting and distance
  
  

  final Color detectedColor = m_colorSensor.getColor();
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getRotations();

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
    //Proximity can be used to make sure consistant distances are being used when imputing blue values
    
    
  }
  public int getRotations() {
  
    return rot;
  }
  

  public void updateRotations(){
    
    if(detectedColor.blue >= BLUE) 
    {
        subRot++;
    }

    if(subRot%2 == 0)
    {
      subRot /= 2;
      rot += subRot;
      subRot = 0;
      //takes amount of time BLUE pops up, and creates a counter. Counter is then converted into revolutions of the wheel
    }
    }
   
}
