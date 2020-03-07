/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package libs.IO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

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
    public Direction direction; //DPAD direction 

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
        direction = this.getDirection(); 
    }

    public static enum Direction {
        NONE(-1), 
        TOP(0), 
        TOP_RIGHT(45), 
        RIGHT(90), 
        BOTTOM_RIGHT(135), 
        BOTTOM(180), 
        BOTTOM_LEFT(225), 
        LEFT(270), 
        TOP_LEFT(315);

        int direction;
        private Direction(int direction) {
            this.direction = direction;
        }
    }

    public Direction getDirection() {
        switch (controller.getPOV()) {
            case 0 : return Direction.TOP;
            case 45 : return Direction.TOP_RIGHT;
            case 90 : return Direction.RIGHT;
            case 135 : return Direction.BOTTOM_RIGHT;
            case 180 : return Direction.BOTTOM;
            case 225 : return Direction.BOTTOM_LEFT;
            case 270 : return Direction.LEFT;
            case 315 : return Direction.TOP_LEFT;
            default : return Direction.NONE;
        }
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
