package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobotBase;//temp
public class GyroTest2 extends TimedRobot {

    private final SPI.Port sPort = SPI.Port.kOnboardCS0;

 final ADXRS450_Gyro gyro = new ADXRS450_Gyro(sPort);


    
@Override
public void robotInit() {



gyro.calibrate();

}
@Override
public void teleopInit() {


    gyro.getAngle();
gyro.reset();
 
//double angle = gyro.getAngle();
SmartDashboard.putNumber("Gyro", gyro.getAngle());
System.out.println(gyro.getAngle());

}

@Override
public void disabledInit() {
    
    super.disabledInit();
}
}