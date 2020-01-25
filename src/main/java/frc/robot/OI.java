/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import libs.IO.ThrustmasterJoystick;
import libs.IO.XboxController;

public class OI {

	public static ThrustmasterJoystick joystick;
	public static XboxController aux;
    
    public OI() {
		joystick = new ThrustmasterJoystick(RobotMap.ActualJoystick);
		aux = new XboxController(RobotMap.JoystickPortXBoxAux);
	}

    //JOYSTICK accessor methods
    public double getX() 
    {
		return joystick.getX();
	}
	
    public double getY() 
    {
		return joystick.getTwist();
	}

    public double getTwist() 
    {
		return joystick.getY();
    }
    
    /**
	 * @return the raw slider value that retuns 0- to +1 insteaad of -1 to +1
	 */
    public double getSlider() 
    {
		return joystick.getSlider();
	}

    //AUX controller accessor methods
    public double getLeftStickYAux() 
    {
		return aux.getLeftStickYAux();
	}
	
    public double getRightStickYAux() 
    {
		return aux.getRightStickYAux();
	}
	
    public double getLeftTriggerAux() 
    {
		return aux.getLeftTriggerAux();
	}
	
    public double getRightTriggerAux() 
    {
		return aux.getRightTriggerAux();
	}

    public void printJoystickChannels() 
    {
		joystick.printJoystickChannels();
	}
}
