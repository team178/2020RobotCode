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
import libs.tof.org.letsbuildrockets.libs.TimeOfFlightSensor;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LawnMower extends SubsystemBase {
  
  private static TalonSRX intake;
  private static TalonSRX conveyorTop;
  private static TalonSRX conveyorBottom;
  private static TalonSRX shooterLeft;
  private static TalonSRX shooterRight;
  private static Solenoid deployer;
  private static Solenoid bouncer;
  private static TimeOfFlightSensor tof1;
  private static TimeOfFlightSensor tof2;
  private static TimeOfFlightSensor tof3;
  private int counter;
  private boolean inTrigger, outTrigger, solenoidTrigger;

  public final double MAX = 150; //These values need to be refined based on the actual robot's dimmensions
  public final double MIN = 60;

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
    solenoidTrigger = false;
  }

  public void ballDump(double speed) {
      moveConveyor(speed);
      shoot(0.9*speed);
  }

  public boolean positionOverride() {
    if (getCounter() < 4) {
      if (tof1.getEdge().equals("No ball") && (tof2.getEdge().equals("Center") || tof2.getEdge().equals("Trailing"))) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

  /* public void runMower(double speed) {
    if (getCounter() < 4) {
      if (!tof1.getEdge().equals("No ball")) {
        intakeBall(speed);
        moveConveyor(speed);
      } else if (tof2.getEdge().equals("Center")) {
        intakeBall(0);
        moveConveyor(0);
      }
    } else {
      intakeBall(0.5*speed); //This might have to be 0...
      moveConveyor(0.5*speed);
    }
  } */

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

  public void extendIntake() {
    deployer.set(true);
  }

  public void retractIntake() {
    deployer.set(false);
  }

  public void startBouncing() {
    bouncer.set(true);
  }

  public void stopBouncing() {
    bouncer.set(false);
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
    while (counter < 0) {
      counter ++;
    }
  }

  public int getCounter() {
    addToCounter();
    removeFromCounter();
    return counter;
  }

  public void periodic() {
    if (Robot.auxController.y.get()) {
      ballDump(0.7);
    } else {
      ballDump(0);
    }

    if (Robot.auxController.getDirection() == Direction.TOP) {
      extendIntake();
    }

    if (Robot.auxController.getDirection() == Direction.BOTTOM) {
      retractIntake();
    }

    if (!positionOverride()) {
      moveConveyor(0.28*Robot.auxController.getLeftStickY());
    } else {
      moveConveyor(0);
    }

    intakeBall(0.4*-Robot.auxController.getRightStickY());
  }
}
