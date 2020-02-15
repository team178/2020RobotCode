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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LawnMower extends SubsystemBase {
  
  private static VictorSPX intake;
  private static Solenoid deployer;
  private static TimeOfFlightSensor tof1;
  private static TimeOfFlightSensor tof2;
  private static TimeOfFlightSensor tof3;
  private int counter;
  private boolean inTrigger, outTrigger, solenoidTrigger;

  public final double MAX = 150; //These values need to be refined based on the actual robot's dimmensions
  public final double MIN = 60;

  public LawnMower() {
    intake = new VictorSPX(RobotMap.intake);
    deployer = new Solenoid(RobotMap.LMdeployer);
    tof1 = new TimeOfFlightSensor(0x623);
    tof2 = new TimeOfFlightSensor(0x620);
    tof3 = new TimeOfFlightSensor(0x624);
    counter = 0;
    inTrigger = true;
    outTrigger = false;
    solenoidTrigger = false;
  }

  public void ballDump(double speed) {
    if (getCounter() != 0) {
      intakeBall(speed);
    } else {
      intakeBall(0);
    }
  }

  public void runMower(double speed) {
    if (getCounter() < 4) {
      if (!tof1.getEdge().equals("No ball")) {
        intakeBall(0.5*speed);
      } else if (tof2.getEdge().equals("Center")) {
        intakeBall(0);
      }
    } else {
      intakeBall(0.5*speed);
    }
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

  public void updateTof1Distance() {
    tof1.updateDistance();
  }

  public void updateTof2Distance() {
    tof2.updateDistance();
  }

  public void updateTof3Distance() {
    tof3.updateDistance();
  }

  public String getTof1Edge() {
    return tof1.getEdge();
  }

  public String getTof2Edge() {
    return tof2.getEdge();
  }

  public String getTof3Edge() {
    return tof3.getEdge();
  }

  public double getTof1Distance() {
    return tof1.getDistance();
  }

  public double getTof2Distance() {
    return tof2.getDistance();
  }

  public double getTof3Distance() {
    return tof3.getDistance();
  }
  
  public void periodic() {

  }
}
