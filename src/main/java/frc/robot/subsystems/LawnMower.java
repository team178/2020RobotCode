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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import libs.IO.XboxController.Direction;
import libs.tof.org.letsbuildrockets.libs.*;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LawnMower extends SubsystemBase {
  
  //Motor controller declaration
  private static TalonSRX intake;
  private static TalonSRX conveyorTop;
  private static TalonSRX conveyorBottom;
  private static TalonSRX shooterLeft;
  private static TalonSRX shooterRight;

  //Pneumatic declaration
  private static Solenoid deployer;
  private static Solenoid bouncer;
  
  //Time of flight declaration
  private static TimeOfFlightSensor tof1;
  private static TimeOfFlightSensor tof2;
  private static TimeOfFlightSensor tof3;

  //Misc
  public int counter;
  private boolean inTrigger, outTrigger;

  public final double MAX = 150;
  public final double MIN = 60;

  private int bouncerCounter = 0;
  private boolean runBouncerCounter = false;

  public LawnMower() {
    intake = new TalonSRX(RobotMap.intake);
    conveyorTop = new TalonSRX(RobotMap.conveyorTop);
    conveyorBottom = new TalonSRX(RobotMap.conveyorBottom);
    shooterLeft = new TalonSRX(RobotMap.shooterLeft);
    shooterRight = new TalonSRX(RobotMap.shooterRight);
    deployer = new Solenoid(RobotMap.LMdeployer);
    bouncer = new Solenoid(RobotMap.LMbouncer);
    tof1 = new TimeOfFlightSensor(0x0623);
    tof2 = new TimeOfFlightSensor(0x0620);
    tof3 = new TimeOfFlightSensor(0x0624);
    counter = 0;
    inTrigger = true;
    outTrigger = false;

    //Set default pneumatic positions
    disableBouncer();
    retractIntake();
  }

  public void ballDump(double conveyorSpeed, double shootSpeed) {
    moveConveyor(conveyorSpeed);
    shoot(shootSpeed);
  }

  public boolean positionOverride() {
    return (tof1.getEdge().equals("No ball") && !tof2.getEdge().equals("No ball")) || (tof1.getEdge().equals("No ball") && tof2.getEdge().equals("No ball"));
  }

  public void intakeBall(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  public void moveConveyor(double speed) {
    conveyorTop.set(ControlMode.PercentOutput, speed);
    conveyorBottom.set(ControlMode.PercentOutput, speed);
  }

  public void shoot(double speed) {
    shooterLeft.set(ControlMode.PercentOutput, -speed);
    shooterRight.set(ControlMode.PercentOutput, -speed);
  }

  public void moveAllMotors(double speed) {
    intakeBall(speed);
    moveConveyor(speed);
    shoot(speed);
  }

  //Pneumatics
  public void extendIntake() {
    deployer.set(true);
  }

  public void enableBouncer() {
    bouncer.set(true); //Robbie was never here idk what you're talking about
  }

  public void retractIntake() {
    deployer.set(false);
  }

  public void disableBouncer() {
    bouncer.set(false);
  }

  //Time of flight distance update
  public void updateTof1Distance() {
    tof1.updateDistance();
  }

  public void updateTof2Distance() {
    tof2.updateDistance();
  }

  public void updateTof3Distance() {
    tof3.updateDistance();
  }

  //Time of flight edge
  public String getTof1Edge() {
    return tof1.getEdge();
  }

  public String getTof2Edge() {
    return tof2.getEdge();
  }

  public String getTof3Edge() {
    return tof3.getEdge();
  }

  //Time of flight distance
  public double getTof1Distance() {
    return tof1.getDistance();
  }

  public double getTof2Distance() {
    return tof2.getDistance();
  }

  public double getTof3Distance() {
    return tof3.getDistance();
  }

  public void addToCounter() {
    if (tof1.getEdge().equals("Leading") && inTrigger) {
      counter++;
      inTrigger = false;
    }
    if (tof1.getEdge().equals("No ball") && !inTrigger) {
      inTrigger = true;
    }
  }

  public void removeFromCounter() {
    if (!(tof3.getEdge().equals("No ball")) && !outTrigger) {
      outTrigger = true;
    }
    if (tof3.getEdge().equals("No ball") && outTrigger) {
      counter--;
      outTrigger = false;
    }
  }

  public void counterFixer() {
    if (counter < 0) {
      counter ++;
    }
  }

  public int getCounter() {
    addToCounter();
    removeFromCounter();
    return counter;
  }

  public void resetCounter() {
    counter = 0;
  }

  @Override
  public void periodic() {
    //Intake pneumatics
    if (Robot.auxController.getDirection() == Direction.TOP) {
      extendIntake();
      runBouncerCounter = true;
    }

    if (Robot.auxController.getDirection() == Direction.BOTTOM) {
      disableBouncer();
      retractIntake();
    } 

    //Bouncer counter
    if (runBouncerCounter) {
      bouncerCounter++;
    }

    if (bouncerCounter >= 100) {
      enableBouncer();
      runBouncerCounter = false;
      bouncerCounter = 0;
    }

    //Intake motors
    if (!Robot.auxController.y.get()) {
      if (!positionOverride()) {
        moveConveyor(0.55 * Robot.auxController.getLeftStickY());
      } else {
        moveConveyor(0);
      }
    }
    
    intakeBall(-0.75 * Robot.auxController.getRightStickY());
  }
}
