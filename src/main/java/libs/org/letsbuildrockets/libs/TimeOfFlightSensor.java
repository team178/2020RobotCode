package libs.org.letsbuildrockets.libs;

import libs.org.letsbuildrockets.libs.CustomCAN;
/**
 * TimeOfFlightSensor
 * V1.0
 */
public class TimeOfFlightSensor {

    int _ID, _distance, _error;
    CustomCAN tofsensor;
    static int TOFCount = 0;

    public TimeOfFlightSensor(int ID) {
        tofsensor = new CustomCAN("TOF"+String.valueOf(TOFCount), ID);
        _ID = ID;
        TOFCount++;
    }

    private void readBuffer() {
        try {
            byte dat[] = tofsensor.readSafely();
            // for (byte byteme : dat) {
                // System.out.printf("rec: 0x%02x\n", byteme);
            // }
            if(dat.length == 3) {
                _error = dat[0] & 0xFF;
                _distance = (dat[1]&0xFF) << 8 | (dat[2]&0xFF);
                // System.out.println("distance: " + _distance);
            }
        } catch (CANMessageUnavailableException e) {
            //System.out.println("CAN error "+e.getMessage());
        }
    }

    public int getDistance() {
        readBuffer();
        return _distance;
    }

    public int getError() {
        readBuffer();
        return _error;
    }

    public boolean inRange() {
        readBuffer();
        return (_error == 0);
    }

}