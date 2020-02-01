package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import libs.org.letsbuildrockets.libs.*;

public class TimeOfFlight extends SubsystemBase {

  private TimeOfFlightSensor tofsensor;
  /**
   * Creates a new Timeofflight.
   */
  public void robotInit() {
    tofsensor = new TimeOfFlightSensor(0x621);
  }

  public double getDistance() {
    // This method will be called once per scheduler run
    if(tofsensor.inRange()){
      System.out.println("distance: " + tofsensor.getDistance()+ " " + tofsensor.getError());
      // distance measured in mm
      if(tofsensor.getDistance() <= 600){
        boolean ballHere = true;
        return 0;
      }
      return tofsensor.getDistance();
    } else {
      System.out.println("out of range");
      return -1;
    }
  }
}

// websites used for background understandng:
// https://docs.wpilib.org/en/latest/docs/software/sensors/encoders-software.html
// https://first.wpi.edu/FRC/roborio/beta/docs/java/edu/wpi/first/wpilibj/Encoder.html