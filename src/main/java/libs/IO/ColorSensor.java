/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package libs.IO;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class ColorSensor {
    private final I2C.Port i2cport = I2C.Port.kOnboard;
    private ColorSensorV3 colorsensor;
    private ColorMatch colormatcher;

    public ColorSensor() {
        colorsensor = new ColorSensorV3(i2cport);
        colormatcher = new ColorMatch();
    }

    public Color detectColor() {
        return colormatcher.matchClosestColor(colorsensor.getColor()).color;
    }

    public double getRed() {
        return detectColor().red;
    }

    public double getGreen() {
        return detectColor().green;
    }

    public double getBlue() {
        return detectColor().blue;
    }

    public int getProximity() {
        return colorsensor.getProximity();
    }

    public int getIR() {
        return colorsensor.getIR();
    }
}
