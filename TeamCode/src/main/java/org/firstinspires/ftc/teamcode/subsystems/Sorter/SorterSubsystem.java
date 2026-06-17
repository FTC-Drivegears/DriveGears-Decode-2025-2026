package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import org.firstinspires.ftc.teamcode.util.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

import java.util.ArrayList;

public class SorterSubsystem {

    private final Hardware hw;
    private final ShooterSubsystem shooterSubsystem;

    private final Servo sorter;
    private final Servo pusher_L;
    private final Servo pusher_R;

    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();
    private final ElapsedTime sorterMoveTimer = new ElapsedTime();

    private static final long SORTER_SETTLE_MS = 75; //200 - 180

    // fix correlatio hetween jnmanual override and turret autoaim
    private static final long SORTER_WRAP_SETTLE_MS = 150; //500 - 400

    private boolean lastMoveWasWrap = false;

    /*
     * MASTER TEST SWITCH.
     * true  -> quickfire ignores selected colour, ball presence, AND shooter
     *          RPM. It just settles, pushes, spins, and repeats across all 3
     *          sorter positions. This is the "push & sort 3 balls no matter
     *          what" timing test path.
     * false -> normal match behaviour: colour-sorted, sensor-confirmed firing
     *          for GREEN/PURPLE, RPM-gated firing for ANY.
     *
     * Flip this to false for real matches.
     */
    private static final boolean TIMING_TEST_MODE = false;

    /*
     * Quickfire sensor-confirmation timing.
     * Quickfire waits for the sorter to settle, then watches the sensors
     * during a small scan window before deciding to move on.
     * (Only used for GREEN / PURPLE — ANY fires straight through.)
     */
    private static final long QUICKFIRE_SCAN_START_MS = 200; //300
    private static final long QUICKFIRE_SCAN_WINDOW_MS = 200; //300

    /*
     * ANY-mode pusher dwell. Pushed to the servo-travel floor (auto-op
     * PUSHER_TIME = 100). GREEN / PURPLE keep the padded 190 / 160 below.
     * Don't drop these below the servo's physical down->up travel or the
     * stroke won't complete.
     */
    private static final long ANY_PUSHER_UP_MS = 75; //100
    private static final long ANY_PUSHER_DOWN_MS = 75; //100

    public SelectedColour selectedColour = SelectedColour.ANY;

    public enum SelectedColour {
        ANY,
        GREEN,
        PURPLE
    }

    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;

    private final Artifact[] sorterList = new Artifact[]{
            new Artifact(Artifact.BallColour.NONE),
            new Artifact(Artifact.BallColour.NONE),
            new Artifact(Artifact.BallColour.NONE)
    };

    private int artifactCount = 0;
    private boolean isPusherUp = false;

    public static final int MAX_NUM_BALLS = 3;

    private int curSorterPositionIndex = 0;

    private final double[] sorterPositions = new double[]{ 0.09, 0.44, 0.82 };

    public QuickfireState quickfireState = QuickfireState.FINISH;

    public enum QuickfireState {
        WAIT_SORT,
        PUSH,
        WAIT_UP,
        DOWN,
        WAIT_DOWN,
        FINISH
    }

    /*
     * Sensor-confirmed quickfire state.
     * This is used for GREEN and PURPLE only.
     */
    private int quickfirePositionsChecked = 0;
    private boolean quickfireSawTargetThisPosition = false;

    /*
     * Simple quickfire: counts how many balls we've fired so we stop
     * after all 3 positions instead of relying on the sensor scan.
     */
    private int quickfireBallsFired = 0;

    /*
     * Quickfire direction: +1 = forward (increasing index), -1 = reverse.
     * Always starts reverse from the last position, so the path is always
     * 2 → 1 → 0.
     */
    private int quickfireDirection = -1;

    public SorterSubsystem(
            Hardware hw,
            ShooterSubsystem shooterSubsystem,
            LinearOpMode opMode,
            Telemetry telemetry,
            String pattern
    ) {
        this.hw = hw;
        this.sorter = hw.sorter;
        this.pusher_R = hw.pusher_R;
        this.pusher_L = hw.pusher_L;
        this.shooterSubsystem = shooterSubsystem;
        this.opMode = opMode;
        this.telemetry = telemetry;

        this.reinitPattern(pattern);
        sorterMoveTimer.reset();
    }

    /*
     * True whenever quickfire should run the simple, gate-free timing path
     * (push & sort all 3 regardless of balls/shooter). Centralised so every
     * call site agrees.
     */
    private boolean useSimplePath() {
        return TIMING_TEST_MODE || selectedColour == SelectedColour.ANY;
    }

    public long getCurrentSettleTimeMs() {
        return lastMoveWasWrap ? SORTER_WRAP_SETTLE_MS : SORTER_SETTLE_MS;
    }
    public double getFirstSorterPos() {
        return sorterPositions[0];
    }

    public double getServoPos() {
        return sorter.getPosition();
    }

    public Artifact[] getSorterList() {
        return sorterList;
    }

    public void setSorterList(Artifact[] newSorterList) {
        artifactCount = 0;

        for (int i = 0; i < MAX_NUM_BALLS; i++) {
            sorterList[i] = newSorterList[i];

            if (!newSorterList[i].is(Artifact.BallColour.NONE)) {
                artifactCount++;
            }
        }
    }

    public void clearSorterList() {
        for (int i = 0; i < MAX_NUM_BALLS; i++) {
            sorterList[i] = new Artifact(Artifact.BallColour.NONE);
        }

        artifactCount = 0;
    }

    public void setCurrentSlotColour(Artifact.BallColour colour) {
        boolean oldWasEmpty = sorterList[curSorterPositionIndex].is(Artifact.BallColour.NONE);
        boolean newIsEmpty = colour == Artifact.BallColour.NONE;

        if (!oldWasEmpty && newIsEmpty) {
            artifactCount = Math.max(0, artifactCount - 1);
        } else if (oldWasEmpty && !newIsEmpty) {
            artifactCount = Math.min(MAX_NUM_BALLS, artifactCount + 1);
        }

        sorterList[curSorterPositionIndex] = new Artifact(colour);
    }

    public int getArtifactCount() {
        return artifactCount;
    }

    public void setArtifactCount(int count) {
        artifactCount = Math.max(0, Math.min(MAX_NUM_BALLS, count));
    }

    public int getSorterPos() {
        return curSorterPositionIndex;
    }

    public boolean isSorterSettling() {
        long settleTime = lastMoveWasWrap ? SORTER_WRAP_SETTLE_MS : SORTER_SETTLE_MS;
        return sorterMoveTimer.milliseconds() < settleTime;
    }

    public void reinitPattern(String pattern) {
        this.pattern = new ArrayList<>();

        for (char p : pattern.toCharArray()) {
            this.pattern.add(p);
        }
    }

    public void manualSpin() {
        if (isPusherUp) return;

        int previousPosition = curSorterPositionIndex;

        curSorterPositionIndex++;
        if (curSorterPositionIndex >= MAX_NUM_BALLS) {
            curSorterPositionIndex = 0;
        }

        lastMoveWasWrap = previousPosition == MAX_NUM_BALLS - 1 && curSorterPositionIndex == 0;

        sorter.setPosition(sorterPositions[curSorterPositionIndex]);
        sorterMoveTimer.reset();
    }

    public void manualSpinReverse() {
        if (isPusherUp) return;

        if (curSorterPositionIndex > 0) {
            curSorterPositionIndex--;
            lastMoveWasWrap = false;
            sorter.setPosition(sorterPositions[curSorterPositionIndex]);
            sorterMoveTimer.reset();
        }
    }

    /*
     * Quickfire-private spin: steps by quickfireDirection and bounces at the
     * ends instead of wrapping. When the next index would go out of bounds the
     * direction flips and the index moves one step in the new direction.
     * lastMoveWasWrap is always false here — a bounce is not a wrap, so we
     * never pay the longer SORTER_WRAP_SETTLE_MS penalty.
     */
    private void quickfireManualSpin() {
        int next = curSorterPositionIndex + quickfireDirection;

        if (next < 0 || next >= MAX_NUM_BALLS) {
            quickfireDirection = -quickfireDirection;
            next = curSorterPositionIndex + quickfireDirection;
        }

        curSorterPositionIndex = next;
        lastMoveWasWrap = false;

        sorter.setPosition(sorterPositions[curSorterPositionIndex]);
        sorterMoveTimer.reset();
    }

    public void removeCurrentBall() {
        if (!sorterList[curSorterPositionIndex].is(Artifact.BallColour.NONE)) {
            sorterList[curSorterPositionIndex] = new Artifact(Artifact.BallColour.NONE);
            artifactCount = Math.max(0, artifactCount - 1);
        }
    }

    public void startQuickfire() {
        if (quickfireState != QuickfireState.FINISH) return;

        quickfirePositionsChecked = 0;
        quickfireSawTargetThisPosition = false;
        quickfireBallsFired = 0;
        quickfireDirection = -1;

        // Always snap to the LAST position before starting so the traversal is
        // always 2 → 1 → 0, visiting every slot exactly once. Without this,
        // starting from the middle (index 1) would go 1 → 0 → bounce → 1 and
        // never reach index 2 before the ball-count limit stopped the sequence.

        curSorterPositionIndex = MAX_NUM_BALLS - 1;
        lastMoveWasWrap = false;
        sorter.setPosition(sorterPositions[MAX_NUM_BALLS - 1]);
        sorterMoveTimer.reset();

        sorterTimer.reset();
        quickfireState = QuickfireState.WAIT_SORT;
    }

    public boolean isActive() {
        return quickfireState != QuickfireState.FINISH;
    }

    public void stopQuickfire() {
        quickfireState = QuickfireState.FINISH;

        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);

        isPusherUp = false;
        quickfirePositionsChecked = 0;
        quickfireSawTargetThisPosition = false;
        quickfireBallsFired = 0;
        quickfireDirection = -1;
    }

    public void quickfireState() {
        quickfireState(Artifact.BallColour.NONE, false);
    }

    public void quickfireState(Artifact.BallColour sensedColour, boolean ballPresent) {
        switch (quickfireState) {
            case WAIT_SORT:
                if (useSimplePath()) {
                    // Timing test / ANY: fire all 3 as fast as possible,
                    // no ball-presence check, no RPM gate.
                    updateSimpleQuickfire();
                } else {
                    updateSensorConfirmedQuickfire(sensedColour, ballPresent);
                }
                break;

            case PUSH:
                pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);

                isPusherUp = true;
                pusherTimer.reset();

                removeCurrentBall();

                quickfireState = QuickfireState.WAIT_UP;
                break;

            case WAIT_UP:
                if (pusherTimer.milliseconds() >= ANY_PUSHER_UP_MS) { //300
                    quickfireState = QuickfireState.DOWN;
                }
                break;

            case DOWN:
                pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);

                isPusherUp = false;
                pusherTimer.reset();

                quickfireState = QuickfireState.WAIT_DOWN;
            case WAIT_DOWN:
                /*
                 * The pusher still needs enough time to physically retract before another
                 * push. The sorter is already moving during this wait.
                 */
                if (pusherTimer.milliseconds() < ANY_PUSHER_DOWN_MS) {
                    break;
                }

                quickfirePositionsChecked = 0;
                quickfireSawTargetThisPosition = false;

                if (useSimplePath()) {
                    quickfireBallsFired++;

                    if (quickfireBallsFired >= MAX_NUM_BALLS) {
                        quickfireState = QuickfireState.FINISH;
                        break;
                    }

                    quickfireManualSpin();
                    sorterTimer.reset();
                    quickfireState = QuickfireState.WAIT_SORT;
                } else {
                    /*
                     * Selected-colour quickfire only fires one matching ball.
                     */
                    quickfireState = QuickfireState.FINISH;
                }
                break;

            case FINISH:
                isPusherUp = false;
                quickfirePositionsChecked = 0;
                quickfireSawTargetThisPosition = false;
                quickfireBallsFired = 0;
                break;
        }
    }

    /*
     * Simple quickfire.
     *
     * The first shot waits for the sorter to settle normally. After each shot,
     * the sorter begins moving at the same time that the pusher is commanded
     * down. Therefore, pusher-down travel and sorter settling overlap.
     */
    private void updateSimpleQuickfire() {
        if (sorterTimer.milliseconds() < getCurrentSettleTimeMs()) return;

        if (!TIMING_TEST_MODE && !shooterSubsystem.isRPMReached()) return;

        quickfireState = QuickfireState.PUSH;
    }

    private void updateSensorConfirmedQuickfire(Artifact.BallColour sensedColour, boolean ballPresent) {
        // Gate on the quickfire-owned sorterTimer instead of sorterMoveTimer,
        // which the colour subsystem keeps resetting.
        if (sorterTimer.milliseconds() < getCurrentSettleTimeMs()) return;

        double elapsed = sorterTimer.milliseconds();

        long scanStart = lastMoveWasWrap ? SORTER_WRAP_SETTLE_MS : QUICKFIRE_SCAN_START_MS;

        if (elapsed < scanStart) return;

        boolean targetDetected = quickfireTargetDetected(sensedColour, ballPresent);

        if (targetDetected) {
            quickfireSawTargetThisPosition = true;
        }

        /*
         * If the correct target has been seen at this position, hold here until
         * RPM is ready, then fire. This prevents moving away just because RPM
         * was late.
         */
        if (quickfireSawTargetThisPosition) {
            if (shooterSubsystem.isRPMReached()) {
                quickfirePositionsChecked = 0;
                quickfireState = QuickfireState.PUSH;
            }
            return;
        }

        /*
         * If the scan window expires and we never saw a valid target,
         * move to the next position.
         */
        if (elapsed >= QUICKFIRE_SCAN_WINDOW_MS) {
            quickfirePositionsChecked++;

            if (quickfirePositionsChecked >= MAX_NUM_BALLS) {
                quickfireState = QuickfireState.FINISH;
                return;
            }

            quickfireManualSpin(); // direction-aware bounce: 2 → 1 → 0
            sorterTimer.reset();
            quickfireSawTargetThisPosition = false;
        }
    }

    private boolean quickfireTargetDetected(Artifact.BallColour sensedColour, boolean ballPresent) {
        if (selectedColour == SelectedColour.ANY) {
            return ballPresent;
        }

        return sensedColour == selectedToBallColour();
    }

    private Artifact.BallColour selectedToBallColour() {
        if (selectedColour == SelectedColour.GREEN) {
            return Artifact.BallColour.GREEN;
        }

        if (selectedColour == SelectedColour.PURPLE) {
            return Artifact.BallColour.PURPLE;
        }

        return Artifact.BallColour.NONE;
    }
}