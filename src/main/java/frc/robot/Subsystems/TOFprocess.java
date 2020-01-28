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

   //basic boolean

  public TOFprocess() {
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  //units are mm

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

    //TOF1 is the first TOF sensor in the intake (faces up)
    //TOF2 is very close to TOF1 and is also in the front of the intake (faces up)
    //TOF3 is mounted in the back of the intake facing towards the power cells coming in
    double ballRadiusMM = 88.9; 

    double TOF1 = 0;
    double TOF2 = 0;
    double TOF3 = 0;
    
    int ballCount = 0;
    
    if(TOF1 == )

  }
  }
}
}


