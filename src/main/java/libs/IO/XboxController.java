/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package libs.IO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Add your docs here.
 */
public class XboxController {

    public static Joystick xbox;
    public static Button auxA;
    public static Button auxB;
    public static Button auxX;
    public static Button auxY;
    public static Button auxLeftBumper;
    public static Button auxRightBumper;
    public static Button auxBack;
    public static Button auxStart;

    public XboxController (int port) {
        xbox = new Joystick(port);
        auxA = new JoystickButton(xbox, 1);
        auxB = new JoystickButton(xbox, 2);
        auxX = new JoystickButton(xbox, 3);
        auxY = new JoystickButton(xbox, 4);
        auxLeftBumper = new JoystickButton(xbox, 5);
        auxRightBumper = new JoystickButton(xbox, 6);
        auxBack = new JoystickButton(xbox, 7);
        auxStart = new JoystickButton(xbox, 8);
    }

    //AUX controller accessor methods
    public double getLeftStickYAux() {
		return xbox.getRawAxis(1);
	}
	
    public double getRightStickYAux() {
		return xbox.getRawAxis(5);
	}
	
    public double getLeftTriggerAux() {
		return xbox.getRawAxis(2);
	}
	
    public double getRightTriggerAux() {
		return xbox.getRawAxis(3);
	}
}
