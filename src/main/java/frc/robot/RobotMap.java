/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public class RobotMap {

    //CAN
    public static int PCM = 0;
    public static int DMLeftMaster = 4;
    public static int DMLeftSlave = 3;
    public static int DMRightMaster = 2;
    public static int DMRightSlave = 1;
    public static int intake = 5;
    public static int conveyorTop = 6;
    public static int conveyorBottom = 7;
    public static int shooterLeft = 8;
    public static int shooterRight = 9;
    public static int leveler = 10;
    public static int winchMaster = 11;
    public static int winchSlave = 12;
    public static int contestant = 13; //jeff ;( 12:50-12:53 T.O.D 2/1/20
    public static int PDP = 13;
    
    public static int encoderFeedbackDevice = 0;
    public static SPI.Port gyroPort = SPI.Port.kOnboardCS0;

    //lights 
    public static int lightsPWM = 3;
    public static int lightsI2CAddress = 4; 
    public static int numOfLEDs = 10;

    //PCM
    public static int LMdeployer = 0;
    public static int LMbouncer = 1;
    public static int hookThurst = 2;
    public static int WOFdeployer = 3;
    
    //Computer USB ports
    public static int ActualJoystick = 0;
    public static int JoystickPortXBoxAux = 1; 
    public static int JoystickBackupMain = 2;
    public static int WiiRemote = 3; 
}
