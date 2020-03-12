package libs.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import libs.limelight.ControlMode.*;
import java.lang.Runnable;

/**
*   Lime Light Class was started by Corey Applegate of Team 3244
*   Granite City Gearheads. We Hope you Enjoy the Lime Light
*   Camera. 
*/
public class LimeLight {

    private NetworkTable table;
    private String name;
    private Boolean isConnected = false;
    private double hearBeatPeriod = 0.1;

    class PeriodicRunnable implements Runnable {

	    public void run() {
            resetPilelineLatency();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(getPipelineLatency()==0.0){
                isConnected = false;
            }else{
                isConnected = true;
            }
        }
    }
    Notifier hearBeat = new Notifier(new PeriodicRunnable());
   
    /**
     * Using the Default Lime Light NT table
     */
    public LimeLight() {
        name = "limelight";
        table = NetworkTableInstance.getDefault().getTable(name);
        hearBeat.startPeriodic(hearBeatPeriod);
    }

    /**
     * If you changed the name of your Lime Light tell Me the New Name
     */
    public LimeLight(String name) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
        hearBeat.startPeriodic(hearBeatPeriod);
    }

    /**
     * Send an instance of the NetworkTabe
     */
    public LimeLight(NetworkTable table) {
        this.table = table;
        hearBeat.startPeriodic(hearBeatPeriod);
    }

    public boolean isConnected() {
        return isConnected;
    }

    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean getIsTargetFound() {
        return table.getEntry("tv").getDouble(0) == 0f;
    }
    /**
     * @return Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    public double getdegRotationToTarget() {
        return table.getEntry("").getDouble(0);
    }
    /**
     * @return vertical offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     */
    public double getdegVerticalToTarget() {
        return table.getEntry("ty").getDouble(0);
    }

    /**
     * @return target area (0% of image to 100% of image)
     */
    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }
    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew_Rotation() {
        return table.getEntry("ts").getDouble(0);
    }

    /**
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     */
    public double getPipelineLatency() {
        return table.getEntry("tl").getDouble(0);
    }

    private void resetPilelineLatency() {
        table.getEntry("tl").setValue(0.0);
    }
    
    /**
     * LedMode  Sets limelight’s LED state
     * 
     *  kon
     *  koff
     *  kblink
     * @param ledMode
     */
    public void setLEDMode(LedMode ledMode) {
        table.getEntry("ledMode").setValue(ledMode.getValue());
    }

    /**
     * @return current LED mode of the Lime Light
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
     * pipeline Sets limelight’s current pipeline
     * 
     * 0 . 9	Select pipeline 0.9
     * 
     * @param pipeline
     */
    public void setPipeline(Integer pipeline) {
        if(pipeline<0){
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        }else if(pipeline>9){
            pipeline = 9;
            throw new IllegalArgumentException("Pipeline can not be greater than nine");
        }
        table.getEntry("pipeline").setValue(pipeline);
    }

    /**
     * Returns current Pipeling of the Lime Light
     * @return Pipelinge
     */
    public double getPipeline() {
        return table.getEntry("pipeline").getDouble(0.0);
    }

    /**
     * @return current Pipeling of the limeilght
     */
    public Integer getPipelineInt() {
        return (int) table.getEntry("pipeline").getDouble(0.0);
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
        NetworkTableEntry stream = table.getEntry("stream");
        double st = stream.getDouble(0.0);
        StreamType mode = StreamType.getByValue(st);
        return mode;
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
     * in normalized screen space (-1 to 1) rather than degrees.	 * 
     */

    public double getAdvanced_RotationToTarget(Advanced_Target raw) {
        NetworkTableEntry txRaw = table.getEntry("tx" + Integer.toString(raw.getValue()));
        double x = txRaw.getDouble(0.0);
        return x;
    }

    public double getAdvanced_degVerticalToTarget(Advanced_Target raw) {
        NetworkTableEntry tyRaw = table.getEntry("ty" + Integer.toString(raw.getValue()));
        double y = tyRaw.getDouble(0.0);
        return y;
    }

    public double getAdvanced_TargetArea(Advanced_Target raw) {
        NetworkTableEntry taRaw = table.getEntry("ta" + Integer.toString(raw.getValue()));
        double a = taRaw.getDouble(0.0);
        return a;
    }
    
    public double getAdvanced_Skew_Rotation(Advanced_Target raw) {
        NetworkTableEntry tsRaw = table.getEntry("ts" + Integer.toString(raw.getValue()));
        double s = tsRaw.getDouble(0.0);
        return s;
    }

    //Raw Crosshairs:
    //If you are using raw targeting data, you can still utilize your calibrated crosshairs:
    
    public double[] getAdvanced_RawCrosshair(Advanced_Crosshair raw){
        double[] crosshars = new double[2];
        crosshars[0] = getAdvanced_RawCrosshair_X(raw);
        crosshars[1] = getAdvanced_RawCrosshair_Y(raw);
        return crosshars;
    }
    public double getAdvanced_RawCrosshair_X(Advanced_Crosshair raw) {
        NetworkTableEntry cxRaw = table.getEntry("cx" + Integer.toString(raw.getValue()));
        double x = cxRaw.getDouble(0.0);
        return x;
    }

    public double getAdvanced_RawCrosshair_Y(Advanced_Crosshair raw) {
        NetworkTableEntry cyRaw = table.getEntry("cy" + Integer.toString(raw.getValue()));
        double y = cyRaw.getDouble(0.0);
        return y;
    }
}
