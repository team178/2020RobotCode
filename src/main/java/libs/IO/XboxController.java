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

    public static Joystick controller;
    public Button a;
    public Button b;
    public Button x;
    public Button y;
    public Button leftBumper;
    public Button rightBumper;
    public Button back;
    public Button start;

    public XboxController(int port) {
        controller = new Joystick(port);
        a = new JoystickButton(controller, 1);
        b = new JoystickButton(controller, 2);
        x = new JoystickButton(controller, 3);
        y = new JoystickButton(controller, 4);
        leftBumper = new JoystickButton(controller, 5);
        rightBumper = new JoystickButton(controller, 6);
        back = new JoystickButton(controller, 7);
        start = new JoystickButton(controller, 8);
    }

    public double getLeftStickX() {
	return controller.getRawAxis(0);
    }
	
    public double getLeftStickY() {
	return controller.getRawAxis(1);
    }
	
    public double getRightStickX() {
	return controller.getRawAxis(4);
    }
	
    public double getRightStickY() {
	return controller.getRawAxis(5);
    }
	
    public double getLeftTrigger() {
	return controller.getRawAxis(2);
    }
	
    public double getRightTrigger() {
	return controller.getRawAxis(3);
    }
}
