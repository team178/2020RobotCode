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
}
