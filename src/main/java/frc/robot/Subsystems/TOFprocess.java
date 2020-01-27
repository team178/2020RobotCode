/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TOFprocess extends SubsystemBase {
  /**
   * Creates a new TOFprocess.
   */
  public TOFprocess() {

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public boolean ballDetected(int TOF) 
  {
    boolean isBall = false;

    if(TOF < 5)
    {
      isBall = true;
    }

    return isBall;
  }

  public int numberOfBalls() 
  {

    int ballCount = 0;

    double TOF1 = 0;
    double TOF2 = 0;
    double TOF3 = 0;
    double distanceMM = 457.2;

    return ballCount;
  }
}
