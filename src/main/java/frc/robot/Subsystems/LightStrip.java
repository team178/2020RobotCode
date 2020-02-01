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
import edu.wpi.first.wpilibj.Timer;

public class LightStrip extends SubsystemBase {
  /**
   * Creates a new Lights.
   */

  public AddressableLED strip;
  public AddressableLEDBuffer lightsColor;
  public int numOfLEDs;
  public DriverStation ds; 
  public Timer timer;

  public LightStrip(int port, int numOfLEDs) {
    this.ds = DriverStation.getInstance();
    this.strip = new AddressableLED(port);
    this.numOfLEDs = numOfLEDs;
    this.lightsColor = new AddressableLEDBuffer(numOfLEDs);
    strip.setLength(lightsColor.getLength());
    strip.setData(lightsColor);
    strip.start();
    this.timer = new Timer();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void forceLights(){
    int red;
    int green;
    int blue;
    int red1;
    int green1;
    int blue1;
    if(ds.getAlliance() != Alliance.Blue){
      red=255;
      green=0;
      blue=0;
    }
    else{
      red=0;
      green=0;
      blue=255;
    }
    for(int i=0; i<10; i++){
      lightsColor.setRGB(i,red,green,blue);
      this.strip.setData(lightsColor);
    }
    int iterators = ((int) Math.random() * numOfLEDs) + 1;
    if(ds.getAlliance() != Alliance.Blue){
      red1=0;
      green1=0;
      blue1=255;
    }
    else{
      red1=255;
      green1=0;
      blue1=0;
    }
    lightsColor.setRGB(iterators, red1, green1, blue1);
  }


  //methods for different lights patterns
  public void lightsaber()//for climber arm 
  {
    timer.start();
    if(ds.getAlliance() != Alliance.Blue)
     {
        for(int i = 0; i < this.numOfLEDs; i++)
        {
            lightsColor.setRGB(i, 99, 0, 0);
            Timer.delay(.25);
            this.strip.setData(lightsColor);
        }
      }
      else 
      {
        for (int i = 0; i < this.numOfLEDs; i++)
        {
            lightsColor.setRGB(i, 31, 235, 253);
            Timer.delay(.25);
            this.strip.setData(lightsColor);
        }
      }
      timer.stop();
    }
  }



