package libs.limelight;

import java.util.HashMap;
import java.util.Map;

public class ControlMode {

    public enum LedMode {
        kPipeline(0),
        kForceOff(1),
        kForceBlink(2),
        kForceOn(3);

        double value;
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
        kVision(0),
        kDriver(1);
        
        double value;
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
        kStandard(0),
        kPiPMain(1),
        kPiPSecondary(2);
        
        double value;
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
        kOff(0),
        kOn(1);
        
        double value;
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
        kOne(0), 
        kTwo(1),
        kThree(2);
        
        int value;
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
        kOne(0),
        kTwo(1);
        
        int value;
        private static final Map<Integer, AdvancedCrosshair> MAP = new HashMap<Integer, AdvancedCrosshair>();
        
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
