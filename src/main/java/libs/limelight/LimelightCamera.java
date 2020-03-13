package libs.limelight;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Notifier;
import libs.limelight.ControlMode.*;

/**
*   Limelight class was started by Corey Applegate of Team 3244
*   Granite City Gearheads. We Hope you Enjoy the Lime Light
*   Camera. 
*/

public class LimelightCamera {

    private NetworkTable table;
    private boolean isConnected = false;
    private double hearBeatPeriod = 0.1;

    class PeriodicRunnable implements Runnable {
	public void run() {
            resetPipelineLatency();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            isConnected = getPipelineLatency() == 0.0;
        }
    }
    Notifier hearBeat = new Notifier(new PeriodicRunnable());
   
    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        hearBeat.startPeriodic(hearBeatPeriod);
    }

    public LimeLight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
        hearBeat.startPeriodic(hearBeatPeriod);
    }

    public LimeLight(NetworkTable table) {
        this.table = table;
        hearBeat.startPeriodic(hearBeatPeriod);
    }

    public boolean isConnected() {
        return isConnected;
    }

    /**
     * @return whether or not the limelight has any valid targets
     */
    public boolean isTargetFound() {
        return table.getEntry("tv").getDouble(0) == 0f;
    }
	
    /**
     * @return the horizontal offset from crosshair to target (-27 to +27 degrees)
     */
    public double getHorizontalDegToTarget() {
        return table.getEntry("tx").getDouble(0);
    }
	
    /**
     * @return vertical offset from crosshair to target (-20.5 to +20.5 degrees)
     */
    public double getVerticalDegToTarget() {
        return table.getEntry("ty").getDouble(0);
    }

    /**
     * @return target area (0% to 100% of image)
     */
    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }
	
    /**
     * @return skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkewRotation() {
        return table.getEntry("ts").getDouble(0);
    }

    /**
     * @return current pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     */
    public double getPipelineLatency() {
        return table.getEntry("tl").getDouble(0);
    }

    private void resetPipelineLatency() {
        table.getEntry("tl").setValue(0.0);
    }
    
    /**
     *  kon
     *  koff
     *  kblink
     * @param ledMode the limelight’s current LED state
     */
    public void setLEDMode(LedMode ledMode) {
        table.getEntry("ledMode").setValue(ledMode.getValue());
    }

    /**
     * @return current LEDMode of the Lime Light
     */
    public LedMode getLEDMode() {
        return LedMode.getByValue(table.getEntry("ledMode").getDouble(0.0));
    }
    
    /**
     * @param camMode sets limelight’s operation mode
     */
    public void setCamMode(CamMode camMode) {
        table.getEntry("camMode").setValue(camMode.getValue());
    }

    /**
     * @return current Cam mode of the Lime Light
     */
    public CamMode getCamMode() {
        return CamMode.getByValue(table.getEntry("camMode").getDouble(0.0));
    }

     /**
     * @param pipeline sets the limelight’s current pipeline
     */
    public void setPipeline(int pipeline) {
        if (pipeline < 0) {
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        } else if(pipeline > 9) {
            pipeline = 9;
            throw new IllegalArgumentException("Pipeline can not be greater than nine");
        }
        table.getEntry("pipeline").setValue(pipeline);
    }

    /**
     * @return current pipeline of limelight
     */
    public double getPipeline() {
        return table.getEntry("pipeline").getDouble(0.0);
    }

    /**
     * @param stream sets limelight’s streaming mode
     * kStandard - Side-by-side streams if a webcam is attached to Limelight
     * kPiPMain - The secondary camera stream is placed in the lower-right corner of the primary camera stream
     * kPiPSecondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
     */ 
    public void setStream(StreamType stream) {
        table.getEntry("stream").setValue(stream.getValue());
    }

    public StreamType getStream() {
        return StreamType.getByValue(table.getEntry("stream").getDouble(0.0));
    }

    /**
     * @param snapshot allows users to take snapshots during a match
     * kon - Stop taking snapshots
     * koff - Take two snapshots per second
     */
    public void setSnapshot(Snapshot snapshot) {
        table.getEntry("snapshot").setValue(snapshot.getValue());
    }

    public Snapshot getSnapshot() {
        return Snapshot.getByValue(table.getEntry("snapshot").getDouble(0.0));
    }

    // *************** Advanced Usage with Raw Contours *********************   

    /**
     * Limelight posts three raw contours to NetworkTables that are not influenced by your grouping mode. 
     * That is, they are filtered with your pipeline parameters, but never grouped. X and Y are returned 
     * in normalized screen space (-1 to 1) rather than degrees.
     */

    public double getAdvancedRotationToTarget(Advanced_Target raw) {
        return table.getEntry("tx" + Integer.toString(raw.getValue())).getDouble(0.0);
    }

    public double getAdvancedDegVerticalToTarget(Advanced_Target raw) {
        return table.getEntry("ty" + Integer.toString(raw.getValue())).getDouble(0.0);
    }

    public double getAdvancedTargetArea(Advanced_Target raw) {
        return table.getEntry("ta" + Integer.toString(raw.getValue())).getDouble(0.0);
    }
    
    public double getAdvancedSkewRotation(Advanced_Target raw) {
        return table.getEntry("ts" + Integer.toString(raw.getValue())).getDouble(0.0);
    }

    //Raw Crosshairs: If you are using raw targeting data, you can still utilize your calibrated crosshairs
    public double[] getAdvancedRawCrosshair(Advanced_Crosshair raw) {
        double[] crosshars = {getAdvancedRawCrosshairX(raw), getAdvancedRawCrosshairY(raw)};
        return crosshars;
    }
	
    public double getAdvancedRawCrosshairX(Advanced_Crosshair raw) {
        return table.getEntry("cx" + Integer.toString(raw.getValue())).getDouble(0.0);
    }

    public double getAdvancedRawCrosshairY(Advanced_Crosshair raw) {
        return table.getEntry("cy" + Integer.toString(raw.getValue())).getDouble(0.0);
    }
}
