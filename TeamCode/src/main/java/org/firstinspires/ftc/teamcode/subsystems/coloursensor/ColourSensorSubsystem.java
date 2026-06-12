package org.firstinspires.ftc.teamcode.subsystems.coloursensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;

/**
 * Threaded ColourSensorSubsystem.
 *
 * The two V3 I2C reads run on a dedicated worker thread and publish their
 * results into volatile fields. The control loop calls update(), which only
 * ever touches those cached fields and runs detection — it NEVER does an I2C
 * read. So a hard V3 bus lock stalls only the worker thread; the main loop
 * keeps ticking and the robot keeps driving and shooting.
 *
 * If the worker hasn't published a fresh read within STALE_MS, colour is
 * treated as down: detection is skipped and sorter state is left alone.
 * Sorting just stops until colour comes back — mobility and shooting are
 * untouched.
 *
 * LIFECYCLE: call stop() when the OpMode ends (right after the while loop).
 * The RC process persists between OpMode runs, so a worker left running
 * leaks a thread (and a stale sensor handle) across runs.
 *
 * Detection logic and public getters are unchanged from the original so the
 * rest of your code (TeleOp, sorter) compiles without edits.
 */
public class ColourSensorSubsystem {

    private static final String TAG = "ColourSensor";

    private static final int  REINIT_THRESHOLD = 10;
    private static final long READ_INTERVAL_MS = 50;   // worker read cadence (~20 Hz)
    private static final long STALE_MS         = 200;  // cache older than this => colour down

    private final Hardware        hw;
    private final HardwareMap     hardwareMap;
    private final SorterSubsystem sorterSubsystem;
    private final Servo           sorter;      // kept for parity; unused here
    private final Servo           leftLight;
    private final Artifact[]      sorterList;

    private NormalizedColorSensor colourSensor1;
    private NormalizedColorSensor colourSensor2;

    // ---- published by the worker thread, read by the main loop ----
    private volatile float   red,  green,  blue,  alpha;
    private volatile float   red2, green2, blue2, alpha2;
    private volatile boolean sensor1Ok  = true;
    private volatile boolean sensor2Ok  = true;
    private volatile long    lastReadMs = 0;

    // ---- worker-thread-only state ----
    private int     failCount1 = 0;
    private int     failCount2 = 0;
    private Thread  worker;
    private volatile boolean running = false;

    // ---- main-thread-only state ----
    private boolean lastArtifactPresent = false;
    private final float alpha_threshold = 0.15f;
    private float lastRed = 0, lastGreen = 0, lastBlue = 0, lastAlpha = 0;

    public ColourSensorSubsystem(HardwareMap hardwareMap, Hardware hw, SorterSubsystem sorterSubsystem) {
        this.hw              = hw;
        this.hardwareMap     = hardwareMap;
        this.sorterSubsystem = sorterSubsystem;
        this.sorter          = hw.sorter;
        this.leftLight       = hw.leftLight;
        this.sorterList      = sorterSubsystem.getSorterList();
        initSensors();
        startWorker();
    }

    private void initSensors() {
        try {
            colourSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colour1");
            sensor1Ok = true; failCount1 = 0;
        } catch (Exception e) {
            colourSensor1 = null; sensor1Ok = false;
            RobotLog.ee(TAG, "colour1 init failed: " + e.getMessage());
        }
        try {
            colourSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colour2");
            sensor2Ok = true; failCount2 = 0;
        } catch (Exception e) {
            colourSensor2 = null; sensor2Ok = false;
            RobotLog.ee(TAG, "colour2 init failed: " + e.getMessage());
        }
    }

    // =====================================================================
    // Worker thread — the ONLY place I2C reads happen
    // =====================================================================

    private void startWorker() {
        running    = true;
        lastReadMs = System.currentTimeMillis();
        worker = new Thread(() -> {
            while (running) {
                readSensor1();
                readSensor2();
                lastReadMs = System.currentTimeMillis();   // stamped only after both reads return
                try { Thread.sleep(READ_INTERVAL_MS); }
                catch (InterruptedException e) { break; }
            }
        }, "ColourReader");
        worker.setDaemon(true);
        worker.start();
    }

    /** Call once when the OpMode ends, after the while loop. */
    public void stop() {
        running = false;
        if (worker != null) worker.interrupt();
    }

    private void readSensor1() {
        if (colourSensor1 == null) { red = green = blue = alpha = 0f; return; }
        try {
            NormalizedRGBA c = colourSensor1.getNormalizedColors();
            red = c.red; green = c.green; blue = c.blue; alpha = c.alpha;
            failCount1 = 0; sensor1Ok = true;
        } catch (Exception e) {
            failCount1++;
            red = green = blue = alpha = 0f;
            RobotLog.ee(TAG, "colour1 read failed (" + failCount1 + "): " + e.getMessage());
            if (failCount1 >= REINIT_THRESHOLD) {
                RobotLog.ww(TAG, "colour1 reinit attempt");
                tryReinit1();
            }
        }
    }

    private void readSensor2() {
        if (colourSensor2 == null) { red2 = green2 = blue2 = alpha2 = 0f; return; }
        try {
            NormalizedRGBA c = colourSensor2.getNormalizedColors();
            red2 = c.red; green2 = c.green; blue2 = c.blue; alpha2 = c.alpha;
            failCount2 = 0; sensor2Ok = true;
        } catch (Exception e) {
            failCount2++;
            red2 = green2 = blue2 = alpha2 = 0f;
            RobotLog.ee(TAG, "colour2 read failed (" + failCount2 + "): " + e.getMessage());
            if (failCount2 >= REINIT_THRESHOLD) {
                RobotLog.ww(TAG, "colour2 reinit attempt");
                tryReinit2();
            }
        }
    }

    private void tryReinit1() {
        try {
            colourSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colour1");
            sensor1Ok = true; failCount1 = 0;
            RobotLog.ii(TAG, "colour1 reinit succeeded");
        } catch (Exception e) {
            colourSensor1 = null; sensor1Ok = false; failCount1 = 0;
            RobotLog.ee(TAG, "colour1 reinit failed: " + e.getMessage());
        }
    }

    private void tryReinit2() {
        try {
            colourSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colour2");
            sensor2Ok = true; failCount2 = 0;
            RobotLog.ii(TAG, "colour2 reinit succeeded");
        } catch (Exception e) {
            colourSensor2 = null; sensor2Ok = false; failCount2 = 0;
            RobotLog.ee(TAG, "colour2 reinit failed: " + e.getMessage());
        }
    }

    // =====================================================================
    // Main loop — cached values only, never blocks on the bus
    // =====================================================================

    public void update(boolean isIntakeMotorOn) {
        boolean stale = (System.currentTimeMillis() - lastReadMs) > STALE_MS;

        // Colour pipeline down (worker wedged, or both sensors gone): leave the
        // sorter alone. Robot keeps driving and shooting; sorting resumes when
        // fresh reads return.
        if (stale || (!sensor1Ok && !sensor2Ok)) {
            lastArtifactPresent = false;
            return;
        }

        if (sorterSubsystem.getArtifactCount() == 3) return;

        // Snapshot the volatiles once so detection sees a consistent frame.
        float r  = red,  g  = green,  b  = blue,  a  = alpha;
        float r2 = red2, g2 = green2, b2 = blue2, a2 = alpha2;

        boolean artifactPresent = a > 0.4f || a2 > 0.4f;
        boolean artifactCleared = a < alpha_threshold && a2 < alpha_threshold;

        boolean purple1 = percentMoreThan(b,  10, r)  && percentMoreThan(b,  10, g);
        boolean purple2 = percentMoreThan(b2, 10, r2) && percentMoreThan(b2, 10, g2);
        boolean green1  = percentMoreThan(g,  20, r)  && percentMoreThan(g,  20, b);
        boolean green2c = percentMoreThan(g2, 20, r2) && percentMoreThan(g2, 20, b2);

        boolean curPosIsEmpty  = sorterList[sorterSubsystem.getSorterPos()].getColour().equals("none");
        boolean notQuickfiring = sorterSubsystem.quickfireState == SorterSubsystem.QuickfireState.FINISH;

        if (notQuickfiring && curPosIsEmpty && artifactPresent && !lastArtifactPresent) {
            if (purple1 || purple2) {
                lastRed = r; lastGreen = g; lastBlue = b; lastAlpha = a;
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Purple");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.7);
                lastArtifactPresent = true;
            } else if (green1 || green2c) {
                lastRed = r; lastGreen = g; lastBlue = b; lastAlpha = a;
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Green");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.5);
                lastArtifactPresent = true;
            }
        }

        if (isIntakeMotorOn && !sorterList[sorterSubsystem.getSorterPos()].getColour().equals("none")) {
            if (sorterSubsystem.getSorterPos() < 3) sorterSubsystem.manualSpin();
        }

        if (artifactCleared) lastArtifactPresent = false;
        else                 lastArtifactPresent = artifactPresent;
    }

    private boolean percentMoreThan(double colour1, double percent, double colour2) {
        return (colour1 / colour2) >= (percent / 100 + 1);
    }

    // =====================================================================
    // Public API (unchanged) + staleness flag
    // =====================================================================

    /** True if the colour worker hasn't produced a fresh read recently (bus wedged). */
    public boolean isStale() { return (System.currentTimeMillis() - lastReadMs) > STALE_MS; }

    public boolean isBallPresent() { return alpha > 0.4f || alpha2 > 0.4f; }
    public boolean isSensor1Ok()   { return sensor1Ok; }
    public boolean isSensor2Ok()   { return sensor2Ok; }

    public float getRed()    { return red;    }
    public float getGreen()  { return green;  }
    public float getBlue()   { return blue;   }
    public float getAlpha()  { return alpha;  }
    public float getRed2()   { return red2;   }
    public float getGreen2() { return green2; }
    public float getBlue2()  { return blue2;  }
    public float getAlpha2() { return alpha2; }
    public float[] getLastValues() { return new float[]{ lastRed, lastGreen, lastBlue, lastAlpha }; }
}