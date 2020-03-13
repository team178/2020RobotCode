package libs.limelight;

import java.util.HashMap;
import java.util.Map;

public class ControlMode {

    public enum LedMode {
        private double value;
        
        kPipeline(0),   //0	use the LED Mode set in the current pipeline
        kForceOff(1),   //1	force off
        kForceBlink(2), //2	force blink
        kForceOn(3);    //3	force on 

        private static final Map<Double, LedMode> MAP = new HashMap<Double, LedMode>();
        
        static {
            for (LedMode ledMode : values()) {
                MAP.put(ledMode.getValue(), ledMode);
            }
        }

        private LedMode(double value) {
            this.value = value;
        }

        public static LedMode getByValue(double value) {
            return MAP.get(value);
        }

        public String toString() {
            return name();
        }
        
        public double getValue() {
            return value;
        }
    }

    
    public enum CamMode {
        private double value;
        
        kVision(0),
        kDriver(1);
    
        private static final Map<Double, CamMode> MAP = new HashMap<Double, CamMode>();
  
        static {
            for (CamMode camMode : values()) {
                MAP.put(camMode.getValue(), camMode);
            }
        }

        private CamMode(double value) {
            this.value = value;
        }

        public static CamMode getByValue(double value) {
            return MAP.get(value);
        }

        public String toString() {
            return name();
        }
        
        public double getValue() {
            return value;
        }
    }

    public enum StreamType {
        private double value;
        
        kStandard(0),
        kPiPMain(1),
        kPiPSecondary(2);
    
        private static final Map<Double, StreamType> MAP = new HashMap<Double, StreamType>();
        
        static {
            for (StreamType streamType : values()) {
                MAP.put(streamType.getValue(), streamType);
            }
        }

        private StreamType(double value) {
            this.value = value;
        }

        public static StreamType getByValue(double value) {
            return MAP.get(value);
        }

        public String toString() {
            return name();
        }
        
        public double getValue() {
            return value;
        }
    }

    public enum Snapshot {
        private double value;
        
        kOff(0),
        kOn(1);
      
        private static final Map<Double, Snapshot> MAP = new HashMap<Double, Snapshot>();
        
        static {
            for (Snapshot snapshot : values()) {
                MAP.put(snapshot.getValue(), snapshot);
            }
        }
      
        private Snapshot(double value) {
            this.value = value;
        }
      
        public static Snapshot getByValue(double value) {
            return MAP.get(value);
        }
      
        public String toString() {
            return name();
        }
        
        public double getValue() {
            return value;
        }
    }
      
    public enum AdvancedTarget {
        private int value;
          
        kOne(0), 
        kTwo(1)
        kThree(2);
      
        private static final Map<Integer, AdvancedTarget> MAP = new HashMap<Integer, AdvancedTarget>();
        
        static {
            for (AdvancedTarget advancedTarget : values()) {
                MAP.put(advancedTarget.getValue(),  advancedTarget);
            }
        }
      
        private AdvancedTarget(int value) {
            this.value = value;
        }
      
        public static AdvancedTarget getByValue(Integer value) {
            return MAP.get(value);
        }
      
        public String toString() {
            return name();
        }
          
        public int getValue() {
            return value;
        }
    }
    
    public enum AdvancedCrosshair {
        private int value;
          
        kOne(0),
        kTwo(1);
      
        private static final Map<Integer, Advanced_Crosshair> MAP = new HashMap<Integer, Advanced_Crosshair>();
        
        static {
            for (AdvancedCrosshair advancedCrosshair : values()) {
                MAP.put(advancedCrosshair.getValue(),  advancedCrosshair);
            }
        }
      
        private AdvancedCrosshair(int value) {
            this.value = value;
        }
      
        public static AdvancedCrosshair getByValue(int value) {
            return MAP.get(value);
        }
      
        public String toString() {
            return name();
        }
        
        public int getValue() {
            return value;
        }
    }
}
