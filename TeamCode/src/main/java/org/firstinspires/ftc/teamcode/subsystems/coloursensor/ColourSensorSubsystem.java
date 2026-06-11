package org.firstinspires.ftc.teamcode.subsystems.coloursensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;

public class ColourSensorSubsystem {

    private static final String TAG = "ColourSensor";

    // How many consecutive failures before we attempt a re-init
    private static final int REINIT_THRESHOLD = 10;

    // Throttle: only hit the I2C bus this often. A sorter does not need colour at
    // full loop rate, and each RevColorSensorV3 read is a synchronous bus
    // transaction that can wedge the bus. Reading ~20 Hz instead of ~30 Hz cuts
    // bus traffic (a known V3 lockup trigger) and halves loop-time exposure to a
    // hang. NOTE: this lowers the probability/frequency of a hang; it does NOT
    // make the read non-blocking — a hard bus lock is a wiring/power/firmware
    // problem, not something user code on a single thread can time out.
    private static final long READ_INTERVAL_MS = 50;
    private final ElapsedTime readTimer = new ElapsedTime();
    private boolean readToggle = false;   // alternate which sensor reads first each cycle

    private boolean lastArtifactPresent = false;
    private Hardware hw;
    private HardwareMap hardwareMap;
    private SorterSubsystem sorterSubsystem;
    private NormalizedColorSensor colourSensor1;
    private NormalizedColorSensor colourSensor2;
    private Servo sorter;
    private Servo leftLight;

    private float alpha_threshold = 0.15f;
    private Artifact[] sorterList;
    private float red, green, blue, alpha;
    private float red2, green2, blue2, alpha2;
    private float lastRed = 0, lastGreen = 0, lastBlue = 0, lastAlpha = 0;

    // Fault tracking per sensor
    private int failCount1 = 0;
    private int failCount2 = 0;
    private boolean sensor1Ok = true;
    private boolean sensor2Ok = true;

    public ColourSensorSubsystem(HardwareMap hardwareMap, Hardware hw, SorterSubsystem sorterSubsystem) {
        this.hw          = hw;
        this.hardwareMap = hardwareMap;
        this.sorterSubsystem = sorterSubsystem;
        this.sorter    = hw.sorter;
        this.leftLight = hw.leftLight;
        this.sorterList = sorterSubsystem.getSorterList();
        initSensors();
        readTimer.reset();
    }

    private void initSensors() {
        try {
            colourSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colour1");
            sensor1Ok = true;
            failCount1 = 0;
        } catch (Exception e) {
            colourSensor1 = null;
            sensor1Ok = false;
            RobotLog.ee(TAG, "colour1 init failed: " + e.getMessage());
        }

        try {
            colourSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colour2");
            sensor2Ok = true;
            failCount2 = 0;
        } catch (Exception e) {
            colourSensor2 = null;
            sensor2Ok = false;
            RobotLog.ee(TAG, "colour2 init failed: " + e.getMessage());
        }
    }

    public void update(boolean isIntakeMotorOn) {
        // Throttle the actual I2C reads + detection. Between reads we keep the last
        // cached colours and do nothing — no bus traffic, no chance to hang.
        if (readTimer.milliseconds() < READ_INTERVAL_MS) return;
        readTimer.reset();

        // Alternate read order each cycle so we don't always do the two bus
        // transactions in the same back-to-back order.
        readToggle = !readToggle;
        if (readToggle) {
            readSensor1();
            readSensor2();
        } else {
            readSensor2();
            readSensor1();
        }

        // If both sensors are gone, bail out — don't touch sorter state
        if (!sensor1Ok && !sensor2Ok) {
            lastArtifactPresent = false;
            return;
        }

        if (sorterSubsystem.getArtifactCount() == 3) return;

        boolean artifactPresent = alpha > 0.4f || alpha2 > 0.4f;
        boolean artifactCleared = alpha < alpha_threshold && alpha2 < alpha_threshold;

        boolean purple_colour1 = percentMoreThan(blue,  10, red)  && percentMoreThan(blue,  10, green);
        boolean purple_colour2 = percentMoreThan(blue2, 10, red2) && percentMoreThan(blue2, 10, green2);
        boolean green_colour1  = percentMoreThan(green,  20, red)  && percentMoreThan(green,  20, blue);
        boolean green_colour2  = percentMoreThan(green2, 20, red2) && percentMoreThan(green2, 20, blue2);

        boolean curPosIsEmpty  = sorterList[sorterSubsystem.getSorterPos()].getColour().equals("none");
        boolean notQuickfiring = sorterSubsystem.quickfireState == SorterSubsystem.QuickfireState.FINISH;

        if (notQuickfiring && curPosIsEmpty && artifactPresent && !lastArtifactPresent) {
            if (purple_colour1 || purple_colour2) {
                lastRed = red; lastGreen = green; lastBlue = blue; lastAlpha = alpha;
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Purple");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.7);
                lastArtifactPresent = true;
            } else if (green_colour1 || green_colour2) {
                lastRed = red; lastGreen = green; lastBlue = blue; lastAlpha = alpha;
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Green");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.5);
                lastArtifactPresent = true;
            }
        }

        if (isIntakeMotorOn && !sorterList[sorterSubsystem.getSorterPos()].getColour().equals("none")) {
            if (sorterSubsystem.getSorterPos() < 3) sorterSubsystem.manualSpin();
        }

        if (artifactCleared) {
            lastArtifactPresent = false;
        } else {
            lastArtifactPresent = artifactPresent;
        }
    }

    private void readSensor1() {
        if (colourSensor1 != null) {
            try {
                NormalizedRGBA colors1 = colourSensor1.getNormalizedColors();
                red   = colors1.red;
                green = colors1.green;
                blue  = colors1.blue;
                alpha = colors1.alpha;
                failCount1 = 0;
                sensor1Ok  = true;
            } catch (Exception e) {
                failCount1++;
                red = green = blue = alpha = 0f;
                RobotLog.ee(TAG, "colour1 read failed (" + failCount1 + "): " + e.getMessage());
                if (failCount1 >= REINIT_THRESHOLD) {
                    RobotLog.ww(TAG, "colour1 reinit attempt");
                    tryReinit1();
                }
            }
        } else {
            red = green = blue = alpha = 0f;
        }
    }

    private void readSensor2() {
        if (colourSensor2 != null) {
            try {
                NormalizedRGBA colors2 = colourSensor2.getNormalizedColors();
                red2   = colors2.red;
                green2 = colors2.green;
                blue2  = colors2.blue;
                alpha2 = colors2.alpha;
                failCount2 = 0;
                sensor2Ok  = true;
            } catch (Exception e) {
                failCount2++;
                red2 = green2 = blue2 = alpha2 = 0f;
                RobotLog.ee(TAG, "colour2 read failed (" + failCount2 + "): " + e.getMessage());
                if (failCount2 >= REINIT_THRESHOLD) {
                    RobotLog.ww(TAG, "colour2 reinit attempt");
                    tryReinit2();
                }
            }
        } else {
            red2 = green2 = blue2 = alpha2 = 0f;
        }
    }

    private void tryReinit1() {
        try {
            colourSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colour1");
            sensor1Ok  = true;
            failCount1 = 0;
            RobotLog.ii(TAG, "colour1 reinit succeeded");
        } catch (Exception e) {
            colourSensor1 = null;
            sensor1Ok  = false;
            failCount1 = 0; // reset so we retry after another REINIT_THRESHOLD failures
            RobotLog.ee(TAG, "colour1 reinit failed: " + e.getMessage());
        }
    }

    private void tryReinit2() {
        try {
            colourSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colour2");
            sensor2Ok  = true;
            failCount2 = 0;
            RobotLog.ii(TAG, "colour2 reinit succeeded");
        } catch (Exception e) {
            colourSensor2 = null;
            sensor2Ok  = false;
            failCount2 = 0;
            RobotLog.ee(TAG, "colour2 reinit failed: " + e.getMessage());
        }
    }

    private boolean percentMoreThan(double colour1, double percent, double colour2) {
        return (colour1 / colour2) >= (percent / 100 + 1);
    }

    /** Returns true if either sensor detects an object. Safe to call if sensors are disconnected. */
    public boolean isBallPresent() { return alpha > 0.4f || alpha2 > 0.4f; }

    public boolean isSensor1Ok()  { return sensor1Ok; }
    public boolean isSensor2Ok()  { return sensor2Ok; }

    public float getRed()    { return red; }
    public float getGreen()  { return green; }
    public float getBlue()   { return blue; }
    public float getAlpha()  { return alpha; }
    public float getRed2()   { return red2; }
    public float getGreen2() { return green2; }
    public float getBlue2()  { return blue2; }
    public float getAlpha2() { return alpha2; }
    public float[] getLastValues() { return new float[]{ lastRed, lastGreen, lastBlue, lastAlpha }; }
}