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

    private static final long SORTER_SETTLE_MS = 250;
    private static final long SORTER_WRAP_SETTLE_MS = 500;

    private boolean lastMoveWasWrap = false;

    /*
     * Quickfire sensor-confirmation timing.
     * Quickfire waits for the sorter to settle, then watches the sensors
     * during a small scan window before deciding to move on.
     */
    private static final long QUICKFIRE_SCAN_START_MS = 300;
    private static final long QUICKFIRE_SCAN_WINDOW_MS = 300;

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
     * This is used for ANY, GREEN, and PURPLE.
     */
    private int quickfirePositionsChecked = 0;
    private boolean quickfireSawTargetThisPosition = false;

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
    }

    public void quickfireState() {
        quickfireState(Artifact.BallColour.NONE, false);
    }

    public void quickfireState(Artifact.BallColour sensedColour, boolean ballPresent) {
        switch (quickfireState) {
            case WAIT_SORT:
                updateSensorConfirmedQuickfire(sensedColour, ballPresent);
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
                if (pusherTimer.milliseconds() >= 300) {
                    quickfireState = QuickfireState.DOWN;
                }
                break;

            case DOWN:
                pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);

                isPusherUp = false;
                pusherTimer.reset();

                quickfireState = QuickfireState.WAIT_DOWN;
                break;

            case WAIT_DOWN:
                if (pusherTimer.milliseconds() >= 400) {
                    quickfirePositionsChecked = 0;
                    quickfireSawTargetThisPosition = false;

                    if (selectedColour == SelectedColour.ANY) {
                        manualSpin();
                        sorterTimer.reset();
                        quickfireState = QuickfireState.WAIT_SORT;
                    } else {
                        quickfireState = QuickfireState.FINISH;
                    }
                }
                break;

            case FINISH:
                isPusherUp = false;
                quickfirePositionsChecked = 0;
                quickfireSawTargetThisPosition = false;
                break;
        }
    }

    private void updateSensorConfirmedQuickfire(Artifact.BallColour sensedColour, boolean ballPresent) {
        if (isSorterSettling()) return;

        double elapsed = sorterTimer.milliseconds();

        long scanStart = lastMoveWasWrap ? SORTER_WRAP_SETTLE_MS : QUICKFIRE_SCAN_START_MS;

        if (elapsed < scanStart) return;

        boolean targetDetected = quickfireTargetDetected(sensedColour, ballPresent);

        if (targetDetected) {
            quickfireSawTargetThisPosition = true;
        }

        /*
         * If the correct target has been seen at this position, hold here until RPM is ready,
         * then fire. This prevents moving away just because RPM was late.
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

            manualSpin();
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