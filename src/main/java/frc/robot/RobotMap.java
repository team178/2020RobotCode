/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
/**
 * Add your docs here.
 */
public class RobotMap {

    //CAN
    public static int PCM = 0;
    public static int DMLeftMaster = 1;
    public static int DMLeftSlave = 2;
    public static int DMRightMaster = 3;
    public static int DMRightSlave = 4;
    public static int intake = 5;
    public static int contestant = 11; //jeff ;( 12:50-12:53 T.O.D 2/1/20

    public static int hookThurst1 = 2;
    public static int hookThrust2= 6;
    public static int winchMaster = 12;
    public static int winchSlave = 13;
    public static int leveler = 14;

    //idk what to do with these
    public static int Encoder1 = 5;
    public static int Encoder2 = 6;
    public static int Encoder3 = 7;
    public static int Encoder4 = 8;

    //PCM
    public static int LMdeployer = 2;
    public static int WOFdeployer = 3;
    
    //Computer USB ports
    public static int ActualJoystick = 0;
    public static int JoystickPortXBoxAux = 1; 
    public static int JoystickPortXBoxMain = 2;

    //Lawnmower
    public static int intakeMotor = 5; 
    public static int intakeDeployer1= 0;
    public static int intakeDeplyer2 = 1;
    public static int timeOfFlightSensor1 = 0x621;
}
