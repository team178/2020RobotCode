/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public final class Constants {

    public static class RobotMap {
        //CAN
        public static int PCM = 0;
        public static int DMLeftMaster = 1;
        public static int DMLeftSlave = 2;
        public static int DMRightMaster = 3;
        public static int DMRightSlave = 4;
        public static int intake = 5;
        
        public static int hookThrust1 = 2;
        public static int hookThrust2= 6;
        public static int contestant = 11; //jeff ;( 12:50-12:53 T.O.D 2/1/20
        public static int winchMaster = 12;
        public static int winchSlave = 13;
        public static int leveler = 14;

        //idk what to do with these
        public static int encoder1 = 5;
        public static int encoder2 = 6;
        public static int encoder3 = 7;
        public static int encoder4 = 8;

        //PCM
        public static int deployerForward = 0;
        public static int deployerReverse = 1;
        
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

    public static class PathConstants {
        //Misc
        public static final double kWheelDiameterMeters = Units.inchesToMeters(6) * Math.PI;
        public static final double kEncoderTicks = 1024;
        public static final double kEncoderDPP = kWheelDiameterMeters / kEncoderTicks;
        public static final double kDriveTolerance = 10e-2;

        public static final double kTrackWidthMeters = Units.inchesToMeters(23.75);
        
        //hannah's comment that she so badly wants me to make
        public static final double kMaxVelMPS = 14;
        public static final double kMaxAccelMPSPS = 8;
        public static final double kMaxVoltage = 11;

        //Feedforward gains
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        //Feedback gains
        public static final double kDriveP = 0;

        //Ramsete
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
    }
}
