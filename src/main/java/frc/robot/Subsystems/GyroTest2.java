package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobotBase;//temp
public class GyroTest2 extends TimedRobot {

    private final SPI.Port sPort = SPI.Port.kOnboardCS1;

 final ADXRS450_Gyro gyro = new ADXRS450_Gyro(sPort);
//public double angle = gyro.getAngle();

    private ADXRS450_Gyro angle;

public void robotInit() {



gyro.reset();
gyro.calibrate();
gyro.getAngle();
gyro.getRate();

}
@Override
public void teleopPeriodic() {
 
//double angle = gyro.getAngle();
SmartDashboard.putNumber("Gyro", angle.getAngle());
System.out.println(angle.getAngle());
}
}