package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroTest2 extends TimedRobot {

    private final SPI.Port sPort = SPI.Port.kMXP;

    final ADXRS450_Gyro gyro = new ADXRS450_Gyro(sPort);

    @Override
    public void robotInit() {
        gyro.calibrate();
        SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
    }
@Override
public void teleopInit() {


    gyro.getAngle();
gyro.reset();


 
//double angle = gyro.getAngle();

System.out.println(gyro.getAngle());

}


}


