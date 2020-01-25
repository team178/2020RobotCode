package org.letsbuildrockets.libs;

import org.letsbuildrockets.libs.CustomCAN;
import org.letsbuildrockets.libs.VersionNumber;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.hal.HAL;
/**
 * TimeOfFlightSensor
 * V1.1
 */
public class TimeOfFlightSensor {

    // Control Bytes
    private static final byte CTRL_SEND_ERROR = 1;
    private static final byte CTRL_SEND_DISTANCE = 2;
    private static final byte CTRL_SET_NEW_ADDR = 3;
    private static final byte CTRL_GET_FIRMWARE_VERSION = 4;
    private static final byte CTRL_SEND_FIRMWARE_VERSION = 5;

    // Error codes
    private static final byte ERROR_NONE = 0;
    private static final byte ERROR_OUT_OF_RANGE = 1;
    private static final byte ERROR_WRITING_TO_CAN = 2;
    private static final byte ERROR_INIT_CAN = 3;
    private static final byte ERROR_INIT_VL53L0X = 4;
    private static final byte ERROR_BAD_CTRL_BYTE = 5;
    private static final byte ERROR_NOT_ENOUGH_DATA_BYTES = 6;

    // Required Firmware
    private static final VersionNumber minVersion =  new VersionNumber(1,1);


    protected int _ID, _distance, _error = -1;
    protected Timer packetTimer;
    protected VersionNumber _firmwareVersion;
    protected CustomCAN tofsensor;
    protected static int TOFCount = 0;

    public TimeOfFlightSensor(int ID) {
        tofsensor = new CustomCAN("TOF"+String.valueOf(TOFCount), ID);
        _firmwareVersion = new VersionNumber(0, 0);
        _ID = ID;
        TOFCount++;
        packetTimer = new Timer();
        packetTimer.start();
        getFirwareVersion();
        if(_firmwareVersion.isOlderThan(minVersion))
            HAL.sendError(true, -2, false, "LBR: Old Firmware! ToF sensor at " + String.format("0x%04x", _ID) + " is on firmware version " + _firmwareVersion.toString() + " but version " + minVersion.toString() + " is required. Upgrade ToF firmware, or downagrade the TimeOfFlight Java library!", "", "", false);    
    }

    private void readBuffer() {
        try {
            byte dat[] = tofsensor.readSafely();
            // for (byte byteme : dat) {
            //     System.out.printf("rec: 0x%02x\n", byteme);
            // }
            switch (dat[0]) {
                case CTRL_SEND_ERROR:
                    if(dat.length == 2){
                        _error = dat[1];
                        if(_error == ERROR_NONE || _error == ERROR_OUT_OF_RANGE)
                            packetTimer.reset();
                    }
                    break;
                case CTRL_SEND_DISTANCE:
                    if(dat.length == 4) {
                        _distance = (dat[1]&0xFF) << 8 | (dat[2]&0xFF);
                        _error = dat[3];
                        //System.out.println("distance: " + _distance);
                        packetTimer.reset();
                    }
                    break;
                case CTRL_SEND_FIRMWARE_VERSION:
                    //System.out.println("ok, we received a firmware number i think");
                    if(dat.length == 3){
                        _firmwareVersion = new VersionNumber(dat[1], dat[2]);
                    }
                    packetTimer.reset();
                    break;
            
                default:
                    break;
            }
        } catch (CANMessageUnavailableException e) {
            //System.out.println("CAN error "+e.getMessage());
            if(packetTimer.hasPeriodPassed(5))
                HAL.sendError(true, -1, false, "LBR: Unable to communicate with ToF sensor at " + String.format("0x%04x", _ID), "", e.getStackTrace().toString(), false);    
        }
    }

    private void sendByte(byte byteme) {
        byte dat[] = new byte[8];
        dat[0] = byteme;
        try {
        tofsensor.writeSafely(dat);
        } catch (UncleanStatusException e) { }
    }

    public void setHarwareCANAddress(int newID) {
        if(newID < 0x0620) {
            System.err.println("The new address for " + tofsensor.getName() + " must be >= 0x0620!");
            return;
        }
        if(newID > 0x0FFF) {
            System.err.println("The new address for " + tofsensor.getName() + " must be <= 0x0FFF!");
            return;
        }
        byte dat[] = new byte[8];
        dat[0] = CTRL_SET_NEW_ADDR;
        dat[1] = (byte)((newID >> 8) & 0xFF);
        dat[2] = (byte)(newID & 0xFF);
        tofsensor.writeSafely(dat);
    }

    public VersionNumber getFirwareVersion() {
        if(_firmwareVersion.major > 0) return _firmwareVersion;
        sendByte(CTRL_GET_FIRMWARE_VERSION);
        for(int i = 0; i < 100; i++) {
            Timer.delay(0.01);
            readBuffer();
            if(_firmwareVersion.major > 0) return _firmwareVersion;
        }
        HAL.sendError(true, -1, false, "LBR: Unable to communicate with ToF sensor at " + String.format("0x%04x", _ID) + " Can't get firmware version!", "", "", false);    
        return _firmwareVersion;
    }

    public int getID() {
        return _ID;
    }

    public int getDistance() {
        if(packetTimer.hasPeriodPassed(0.01))
            readBuffer();
        return _distance;
    }

    public int getError() {
        if(packetTimer.hasPeriodPassed(0.01))
            readBuffer();
        return _error;
    }

    public boolean inRange() {
        if(packetTimer.hasPeriodPassed(0.01))
            readBuffer();
        return (_error == 0);
    }

}