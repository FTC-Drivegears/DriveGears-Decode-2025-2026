package org.firstinspires.ftc.teamcode.subsystems.coloursensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;

public class ColourSensorSubsystem {

    private static final String TAG = "ColourSensor";

    private static final int REINIT_THRESHOLD = 10;

    /*
     * TUNE THESE:
     */
    private static final long READ_INTERVAL_MS = 50;
    private static final long ADD_COOLDOWN_MS = 100;

    private static final float BALL_PRESENT_ALPHA_THRESHOLD = 0.25f;
    private static final float BALL_CLEAR_ALPHA_THRESHOLD   = 0.15f;

    private static final int BALL_PRESENT_CONFIRM_READS = 2;
    private static final int BALL_CLEAR_CONFIRM_READS   = 3;

    private static final double PURPLE_BLUE_MARGIN_PERCENT = 10.0;
    private static final double GREEN_GREEN_MARGIN_PERCENT = 10.0;

    private final ElapsedTime readTimer = new ElapsedTime();
    private final ElapsedTime addCooldownTimer = new ElapsedTime();

    private boolean readToggle = false;
    private boolean lastArtifactPresent = false;

    private int presentReadCount = 0;
    private int clearReadCount = 0;

    private final Hardware hw;
    private final HardwareMap hardwareMap;
    private final SorterSubsystem sorterSubsystem;

    private NormalizedColorSensor colourSensor1;
    private NormalizedColorSensor colourSensor2;

    private final Artifact[] sorterList;

    private float red, green, blue, alpha;
    private float red2, green2, blue2, alpha2;
    private float lastRed = 0, lastGreen = 0, lastBlue = 0, lastAlpha = 0;

    private int failCount1 = 0;
    private int failCount2 = 0;

    private boolean sensor1Ok = true;
    private boolean sensor2Ok = true;

    public Artifact.BallColour currentBallColour = Artifact.BallColour.NONE;

    public ColourSensorSubsystem(HardwareMap hardwareMap, Hardware hw, SorterSubsystem sorterSubsystem) {
        this.hw = hw;
        this.hardwareMap = hardwareMap;
        this.sorterSubsystem = sorterSubsystem;
        this.sorterList = sorterSubsystem.getSorterList();

        initSensors();
        readTimer.reset();
        addCooldownTimer.reset();
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
        if (readTimer.milliseconds() < READ_INTERVAL_MS) return;
        readTimer.reset();

        readBothSensors();

        if (!sensor1Ok && !sensor2Ok) {
            currentBallColour = Artifact.BallColour.NONE;
            lastArtifactPresent = false;
            presentReadCount = 0;
            clearReadCount = 0;
            return;
        }

        boolean rawPresent = isRawBallPresent();
        boolean rawClear = isRawBallCleared();

        updatePresenceDebounce(rawPresent, rawClear);

        boolean artifactPresent = presentReadCount >= BALL_PRESENT_CONFIRM_READS;
        boolean artifactCleared = clearReadCount >= BALL_CLEAR_CONFIRM_READS;

        currentBallColour = detectCurrentBallColour(rawPresent);

        /*
         * Do not change sorter list while sorter is moving.
         */
        if (sorterSubsystem.isSorterSettling()) {
            if (artifactCleared) {
                lastArtifactPresent = false;
                currentBallColour = Artifact.BallColour.NONE;
            } else {
                lastArtifactPresent = artifactPresent;
            }
            return;
        }

        boolean notQuickfiring = sorterSubsystem.quickfireState == SorterSubsystem.QuickfireState.FINISH;
        boolean curPosIsEmpty = sorterList[sorterSubsystem.getSorterPos()].is(Artifact.BallColour.NONE);
        boolean addCooldownFinished = addCooldownTimer.milliseconds() >= ADD_COOLDOWN_MS;

        /*
         * Normal intake update is ADD-ONLY.
         * It does not delete balls just because the sensor briefly sees NONE.
         */
        if (
                notQuickfiring
                        && artifactPresent
                        && !lastArtifactPresent
                        && curPosIsEmpty
                        && addCooldownFinished
        ) {
            if (currentBallColour == Artifact.BallColour.PURPLE) {
                rememberLastSensorValues();
                sorterSubsystem.setCurrentSlotColour(Artifact.BallColour.PURPLE);
                lastArtifactPresent = true;
                addCooldownTimer.reset();
            } else if (currentBallColour == Artifact.BallColour.GREEN) {
                rememberLastSensorValues();
                sorterSubsystem.setCurrentSlotColour(Artifact.BallColour.GREEN);
                lastArtifactPresent = true;
                addCooldownTimer.reset();
            }
        }

        /*
         * Auto-spin only if intake is on, the current slot is filled,
         * and the sorter is not already full.
         */
        if (
                isIntakeMotorOn
                        && sorterSubsystem.getArtifactCount() < SorterSubsystem.MAX_NUM_BALLS
                        && !sorterList[sorterSubsystem.getSorterPos()].is(Artifact.BallColour.NONE)
        ) {
            sorterSubsystem.manualSpin();
        }

        if (artifactCleared) {
            lastArtifactPresent = false;
            currentBallColour = Artifact.BallColour.NONE;
        } else {
            lastArtifactPresent = artifactPresent;
        }
    }

    /*
     * Used by rescan and quickfire.
     * This intentionally uses a direct raw read so quickfire can check the actual
     * current physical position.
     */
    public Artifact.BallColour sampleCurrentBallColour() {
        readBothSensors();

        if (!sensor1Ok && !sensor2Ok) {
            currentBallColour = Artifact.BallColour.NONE;
            lastArtifactPresent = false;
            return Artifact.BallColour.NONE;
        }

        boolean rawPresent = isRawBallPresent();

        currentBallColour = detectCurrentBallColour(rawPresent);

        if (currentBallColour == Artifact.BallColour.NONE) {
            lastArtifactPresent = false;
        } else {
            lastArtifactPresent = true;
            rememberLastSensorValues();
        }

        return currentBallColour;
    }

    private void readBothSensors() {
        readToggle = !readToggle;

        if (readToggle) {
            readSensor1();
            readSensor2();
        } else {
            readSensor2();
            readSensor1();
        }
    }

    private void updatePresenceDebounce(boolean rawPresent, boolean rawClear) {
        if (rawPresent) {
            presentReadCount++;
            clearReadCount = 0;
        } else if (rawClear) {
            clearReadCount++;
            presentReadCount = 0;
        }
    }

    private boolean isRawBallPresent() {
        return alpha > BALL_PRESENT_ALPHA_THRESHOLD || alpha2 > BALL_PRESENT_ALPHA_THRESHOLD;
    }

    private boolean isRawBallCleared() {
        return alpha < BALL_CLEAR_ALPHA_THRESHOLD && alpha2 < BALL_CLEAR_ALPHA_THRESHOLD;
    }

    private Artifact.BallColour detectCurrentBallColour(boolean artifactPresent) {
        if (!artifactPresent) {
            return Artifact.BallColour.NONE;
        }

        boolean purple_colour1 = percentMoreThan(blue, PURPLE_BLUE_MARGIN_PERCENT, red)
                && percentMoreThan(blue, PURPLE_BLUE_MARGIN_PERCENT, green);

        boolean purple_colour2 = percentMoreThan(blue2, PURPLE_BLUE_MARGIN_PERCENT, red2)
                && percentMoreThan(blue2, PURPLE_BLUE_MARGIN_PERCENT, green2);

        boolean green_colour1 = percentMoreThan(green, GREEN_GREEN_MARGIN_PERCENT, red)
                && percentMoreThan(green, GREEN_GREEN_MARGIN_PERCENT, blue);

        boolean green_colour2 = percentMoreThan(green2, GREEN_GREEN_MARGIN_PERCENT, red2)
                && percentMoreThan(green2, GREEN_GREEN_MARGIN_PERCENT, blue2);

        if (purple_colour1 || purple_colour2) {
            return Artifact.BallColour.PURPLE;
        }

        if (green_colour1 || green_colour2) {
            return Artifact.BallColour.GREEN;
        }

        return Artifact.BallColour.NONE;
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
            sensor1Ok = true;
            failCount1 = 0;
            RobotLog.ii(TAG, "colour1 reinit succeeded");
        } catch (Exception e) {
            colourSensor1 = null;
            sensor1Ok = false;
            failCount1 = 0;
            RobotLog.ee(TAG, "colour1 reinit failed: " + e.getMessage());
        }
    }

    private void tryReinit2() {
        try {
            colourSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colour2");
            sensor2Ok = true;
            failCount2 = 0;
            RobotLog.ii(TAG, "colour2 reinit succeeded");
        } catch (Exception e) {
            colourSensor2 = null;
            sensor2Ok = false;
            failCount2 = 0;
            RobotLog.ee(TAG, "colour2 reinit failed: " + e.getMessage());
        }
    }

    private boolean percentMoreThan(double colour1, double percent, double colour2) {
        if (colour2 <= 0.0001) return colour1 > 0.0001;
        return (colour1 / colour2) >= (percent / 100.0 + 1.0);
    }

    private void rememberLastSensorValues() {
        lastRed = red;
        lastGreen = green;
        lastBlue = blue;
        lastAlpha = alpha;
    }

    public boolean isBallPresent() {
        return isRawBallPresent();
    }

    public Artifact.BallColour getCurrentBallColour() {
        return currentBallColour;
    }

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

    public float[] getLastValues() {
        return new float[]{ lastRed, lastGreen, lastBlue, lastAlpha };
    }
}