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

    // Team 178 2020 Specific Fields
    private double[] values;
    private String lastEdge;

    public final double MAX = 150; //These values need to be refined based on the actual dimmensions of the lawnmower
    public final double MIN = 60;

    public TimeOfFlightSensor(int ID) {
        tofsensor = new CustomCAN("TOF"+String.valueOf(TOFCount), ID);
        _ID = ID;
        TOFCount++;       
        values = new double[2];
        lastEdge = "None";
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

        //Team 178 2020 Specific Methods
        public void updateDistance() {
            values[1] = values[0];
            values[0] = getDistance();
        }
    
        public String getEdge() {
            double secant = (values[1] - values[0])/0.02;
            if (values[0] > MAX) {
                return "No ball";
            } else if (values[0] < MIN) {
                return "Center";
            } else if (secant > 100) {
                lastEdge = "Leading";
                return "Leading";
            } else if (secant < -100) {
                lastEdge = "Trailing";
                return "Trailing";
            } else if (lastEdge == "Leading") {
                return "Leading";
            } else if (lastEdge == "Trailing") {
                return "Trailing";
            } else if (lastEdge == "None") {
                return "No ball";
            }
            return "No ball";
        }
}