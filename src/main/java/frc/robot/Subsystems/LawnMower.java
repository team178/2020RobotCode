/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import libs.org.letsbuildrockets.libs.TimeOfFlightSensor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LawnMower extends SubsystemBase {
  
  private static VictorSPX intake;
  private static DoubleSolenoid deployer;
  private static TimeOfFlightSensor tof1;
  private static TimeOfFlightSensor tof2;
//  private static TimeOfFlightSensor tof3;
  private static double tof1Previous;
  private static double tof2Previous;
  private static boolean firstTime;

  public LawnMower() {
    intake = new VictorSPX(RobotMap.intake);
    deployer = new DoubleSolenoid(RobotMap.deployerForward, RobotMap.deployerReverse);
    tof1 = new TimeOfFlightSensor(0x621);
    tof2 = new TimeOfFlightSensor(0x622);
//    tof3 = new TimeOfFlightSensor(0x623);
    tof1Previous = 0;
    tof2Previous = 0;
    firstTime = true;
  }

  public void intakeBall (double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void extendIntake() {
    deployer.set(DoubleSolenoid.Value.kForward); // Might be kReverse, test
  }

  public void retractIntake() {
    deployer.set(DoubleSolenoid.Value.kReverse); // Might be kForward, test
  }

  public double getTof1Distance() {
    if(tof1.inRange()){
      System.out.println("distance: " + tof1.getDistance()+ " " + tof1.getError());
      // distance measured in mm
      if(tof1.getDistance() <= 600){
        boolean ballHere = true;
        return 0;
      }
      return tof1.getDistance();
    } else {
      System.out.println("out of range");
      return -1;
    }
  }

  public double getTof2Distance() {
    // This method will be called once per scheduler run
    if(tof2.inRange()){
      System.out.println("distance: " + tof2.getDistance()+ " " + tof2.getError());
      // distance measured in mm
      if(tof2.getDistance() <= 100){
        boolean ballHere = true;
        return 0;
      }
      return tof2.getDistance();
    } else {
      System.out.println("out of range");
      return -1;
    }
  }

  public String tof1Edge() {
    double secant = ((getTof1Distance() - tof1Previous)/0.2);
    if (secant > 10) { //check this value
      return "Trailing";
    } else if (secant < 10) { //check this value
      return "Leading";
    } else {
      return "Middle";
    }
  }

  public String tof2Edge() {
    double secant = ((getTof2Distance() - tof2Previous)/0.2);
    if (secant > 10) { //check this value
      return "Trailing";
    } else if (secant < 10) {
      return "Leading";
    } else {
      return "Middle";
    }
  }

/*  public double getTof3Distnace() {
    // This method will be called once per scheduler run
    if(tof3.inRange()){
      System.out.println("distance: " + tof3.getDistance()+ " " + tof3.getError());
      // distance measured in mm
      if(tof3.getDistance() <= 600){
        boolean ballHere = true;
        return 0;
      }
      return tof3.getDistance();
    } else {
      System.out.println("out of range");
      return -1;
    }
  } */

  public void periodic() {

  }
}
