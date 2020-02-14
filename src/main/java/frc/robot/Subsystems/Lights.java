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


public class Lights extends SubsystemBase {
  /**
   * Creates a new Lights.
   */
  protected I2C arduino;
  protected DriverStation ds;
  protected boolean received;
  protected boolean sent;

  public Lights(Port port, int address) {
    arduino = new I2C(port, address);
    ds = DriverStation.getInstance();
  }


  public boolean sendMessage(String pattern) {
    boolean sent = false;
    String message = pattern.toLowerCase();
    
    //System.out.println("Message Sent: " + message);
    sent = !arduino.writeBulk(message.getBytes()); //Returns true if aborted, false if completed
    //System.out.println("Message Sent? " + arduino.addressOnly());

    return sent;
  }

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

  //Checker methods
  public boolean checkIfReceived()
  {
    return received;
  }

  public boolean checkIfSent()
  {
    return sent;
  }
  
  //Lights methods -- sends characters to arduino to indicate different light colors
  public boolean setAllianceColor() {
    if (ds.getAlliance() == Alliance.Blue) {
      return sendMessage("b");
    }
    else {
      return sendMessage("r");
    }
  }

  public boolean lights1() {
    return sendMessage("h");
  }

  public boolean lights2() {
    return sendMessage("c");
  }

  public boolean lightsOff()
  {
    return sendMessage("n");
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
