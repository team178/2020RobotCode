/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class LightsArduino extends SubsystemBase {
  /**
   * Creates a new Lights object, comprised of an Arduino connected via I2C. 
   */
  protected I2C arduino;
  protected DriverStation ds;
  protected boolean received;
  protected boolean sent;
/**
 * Creates a new LightsArduino, taking an i2c port number and i2c address
 * @param port
 * @param address
 */
  public LightsArduino(Port port, int address) {
    arduino = new I2C(port, address);
    ds = DriverStation.getInstance();
  }


  
  /** 
   * @return boolean
   */
   public boolean sendMessage(String pattern) {
    boolean sent = false;
    String message = pattern.toLowerCase();
    
   System.out.println("Message Sent: " + message);
    sent = !arduino.writeBulk(message.getBytes()); //Returns true if aborted, false if completed
    System.out.println("Message Sent? " + arduino.addressOnly());

    return sent;
  }
  
public boolean sendMessage(char message){
  byte[] bytearray=new byte[1];
  bytearray[0]=(byte)message;
  sent=!arduino.writeBulk(bytearray);
  return sent;
}
  
  /** 
   * @param address
   * @return byte[]
   */
  public byte[] receiveMessage(int address) //for which i2c address to read from
  {
    byte[] dataFromArduino = new byte[2]; //change based on type of data 
    received = !arduino.read(address, 2, dataFromArduino); //Returns true if aborted, false if completed
    //System.out.print("Message Received: ");
    
    for (byte b : dataFromArduino) { //Receives data in bytes from arduino and converts to binary 
      String s1 = String.format("%8s", Integer.toBinaryString(b & 0xFF)).replace(' ', '0');
      //System.out.print(s1 + ", ");
    }
    //System.out.println();
    
    return dataFromArduino;
  }

  
  /** 
   * @return boolean
   */
  //Checker methods
  public boolean checkIfReceived()
  {
    return received;
  }

  
  /** 
   * @return boolean
   */
  public boolean checkIfSent()
  {
    return sent;
  }
  
  
  /** 
   * @return boolean
   */
  //Lights methods -- sends characters to arduino to indicate different light colors
  public boolean setAllianceColor() {
    if (ds.getAlliance() == Alliance.Blue) {
      return blue();
    }
    else {
      return red();
    }
  }

  
  /** 
   * @return boolean
   */
  public boolean red() {
    //System.out.println("red");
    return sendMessage('r');
  }

  
  /** 
   * @return boolean
   */
  public boolean blue() {
    //System.out.println("blue");
    return sendMessage('b');
  }

  
  /** 
   * @return boolean
   */
  public boolean off()
  {
    //System.out.println("off");
    return sendMessage('n');
  }

  public boolean ball(int balls)
  {
    int bin = Integer.parseInt(Integer.toBinaryString(balls));
    char output = (char)bin;
    //System.out.println(output);
    return sendMessage(output);
  }


  @Override
  public void periodic() {
    ball(Robot.lawnmower.getCounter());
    setAllianceColor();
    
  }
}
