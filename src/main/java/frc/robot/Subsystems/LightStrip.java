/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;//do driverstation stuff in Robot.java 
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LightStrip extends SubsystemBase {
  /**
   * Creates a new Lights.
   */

  public AddressableLED strip;
  public AddressableLEDBuffer lightsColor;
  public int numOfLEDs;
  public DriverStation ds; 

  public LightStrip(int port, int numOfLEDs) {
    this.ds = DriverStation.getInstance();
    this.strip = new AddressableLED(port);
    this.numOfLEDs = numOfLEDs;
    this.lightsColor = new AddressableLEDBuffer(numOfLEDs);
    strip.setLength(lightsColor.getLength());
    strip.setData(lightsColor);
    strip.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }


  //methods for different lights patterns
  public void lightsaber()//for climber arm 
  {
    if(ds.getAlliance() != Alliance.Blue)
     {
        for(int i = 0; i < this.numOfLEDs; i++)
        {
            lightsColor.setRGB(i, 99, 0, 0);
            //delay
            this.strip.setData(lightsColor);
        }
      }
      else 
      {
        for (int i = 0; i < this.numOfLEDs; i++)
        {
          lightsColor.setRGB(i, 31, 235, 253);
          //delay
          this.strip.setData(lightsColor);
        }
      }
    }

  }



