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
 * When holding the remote horizontally with Wii logo facing upward, tilting forward and backward will let you drive the robot accordingly 
 * */
public class WiiRemote {

    public static Joystick wiimote; 

    //main wiimote buttons 
    public Button a; 
    public Button b; 
    public Button plus; 
    public Button minus; 
    public Button home; 
    public Button one; 
    public Button two; 

    //nunchuck buttons 
    public Button c; 
    public Button z;

    //classic controller buttons 
    //x,y,l,r,zl,zr

    //can add DPAD code too

    public boolean isNunchuck; 

    public WiiRemote (int port, boolean isNunchuck)
    {
        wiimote = new Joystick(port);

        if(isNunchuck)
        {
            a = new JoystickButton(wiimote, 1);
            b = new JoystickButton(wiimote, 2);
            c = new JoystickButton(wiimote, 3);
            z = new JoystickButton(wiimote, 4);
            one = new JoystickButton(wiimote, 5);
            two = new JoystickButton(wiimote, 6);
            plus = new JoystickButton(wiimote, 7);
            minus = new JoystickButton(wiimote, 8);
        }
        else
        {
            a = new JoystickButton(wiimote, 3);
            b = new JoystickButton(wiimote, 4);
            plus = new JoystickButton(wiimote, 5);
            minus = new JoystickButton(wiimote, 6);
            one = new JoystickButton(wiimote, 1);
            two = new JoystickButton(wiimote, 2); 
        }
    }

    public double getXRot()
    {
        return wiimote.getRawAxis(3);
    }

    public double getYRot()//inverted (forward tilt is 0)
    {
        return wiimote.getRawAxis(4);
    }

    public double getNunchuckX()
    {
        return wiimote.getRawAxis(1);
    }

    public double getNunchuckY()
    {
        return wiimote.getRawAxis(2);
    }

    
}
