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
public class Constants {

    //Drive constants
    public static double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(6) * Math.PI;
    public static double ENCODER_TICKS = 1024;
    public static double TRACK_WIDTH_INCHES = 23.75;
    public static double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH_INCHES);
    public static double ENCODER_DPP = WHEEL_CIRCUMFRENCE / ENCODER_TICKS;

    //Pathplanning constants
    public static double MAX_VELOCITY_MPS = 0;
    public static double MAX_ACCEL_MPSPS = 0;
    public static double RAMSETE_B = 0;
    public static double RAMSETE_ZETA = 0;
    public static double OPTIMAL_DRIVE_KP = 0;
}
