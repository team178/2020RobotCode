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
  private double[] tof1Values;
  private double[] tof2Values;
  private String tof1LastEdge;
  private String tof2LastEdge;

  public final double MAX = 150; //These values need to be refined based on the actual robot's dimmensions
  public final double MIN = 60;

  public LawnMower() {
    intake = new VictorSPX(RobotMap.intake);
    deployer = new DoubleSolenoid(RobotMap.deployerForward, RobotMap.deployerReverse);
    tof1 = new TimeOfFlightSensor(0x621);
    tof2 = new TimeOfFlightSensor(0x622);
//    tof3 = new TimeOfFlightSensor(0x623);
    tof1Previous = 0;
    tof2Previous = 0;
    firstTime = true;
    tof1Values = new double[2];
    tof2Values = new double[2];
    tof1LastEdge = "None";
    tof2LastEdge = "None";
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
    tof1Values[1] = tof1Values[0];
    tof1Values[0] = tof1.getDistance();
    return tof1Values[0];
  }

  public String getTof1Edge() {
    double secant = (tof1Values[1] - tof1Values[0])/0.02;
    //System.out.println("Slope of Secant Line: " + secant);
    if (tof1Values[0] > MAX) { //test this max value
      return "No ball";
    } else if (tof1Values[0] < MIN) { //test this min value
      return "Center";
    } else if (secant > 100) {
      tof1LastEdge = "Leading";
      return "Leading";
    } else if (secant < -100) {
      tof1LastEdge = "Trailing";
      return "Trailing";
    } else if (tof1LastEdge == "Leading") {
      return "Leading";
    } else if (tof1LastEdge == "Trailing") {
      return "Trailing";
    } else if (tof1LastEdge == "None") {
      return "No ball";
    }
    return "No ball";
    //A minimum value returns "Center", a maximum value returns "No Ball"
  }

  public double getTof2Distance() {
    tof2Values[1] = tof2Values[0];
    tof2Values[0] = tof2.getDistance();
    return tof2Values[0];
  }

  public String getTof2Edge() {
    double secant = (tof2Values[1] - tof2Values[0])/0.02;
    //System.out.println("Slope of Secant Line: " + secant);
    if (tof2Values[0] > MAX) { //test this max value
      return "No ball";
    } else if (tof2Values[0] < MIN) { //test this min value
      return "Center";
    } else if (secant > 100) {
      tof2LastEdge = "Leading";
      return "Leading";
    } else if (secant < -100) {
      tof2LastEdge = "Trailing";
      return "Trailing";
    } else if (tof2LastEdge == "Leading") {
      return "Leading";
    } else if (tof2LastEdge == "Trailing") {
      return "Trailing";
    } else if (tof2LastEdge == "None") {
      return "No ball";
    }
    return "No ball";
    //A minimum value returns "Center", a maximum value returns "No Ball"
  }

  public void periodic() {

  }
}
