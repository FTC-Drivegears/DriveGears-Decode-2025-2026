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

    // ---------------- SUBSYSTEMS ----------------
    private Hardware hw;
    private ShooterSubsystem shooterSubsystem;

    // ---------------- HARDWARE ----------------
    private final Servo sorter;
    private Servo pusher_L;
    private Servo pusher_R;

    // ---------------- TIMERS ----------------
    private static final ElapsedTime stageTimer  = new ElapsedTime();
    private final ElapsedTime sorterTimer         = new ElapsedTime();
    private final ElapsedTime pusherTimer         = new ElapsedTime();

    // ---------------- COLOUR SELECTION ----------------
    public SelectedColour selectedColour = SelectedColour.ANY;

    public enum SelectedColour {
        ANY,
        GREEN,
        PURPLE
    }

    // ---------------- STATE ----------------
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;

    private final Artifact[] sorterList = new Artifact[]{
            new Artifact("none"), new Artifact("none"), new Artifact("none")
    };
    private int[] artifactCount = new int[]{ 0 };
    private boolean isPusherUp = false;
    public static final int MAX_NUM_BALLS = 3;

    private int curSorterPositionIndex = 0;
    // Position 1 changed from 0.42 → 0.43 to match physical slot
    private final double[] sorterPositions = new double[]{ 0.09, 0.44, 0.82 };

    public SorterSubsystem(Hardware hw, ShooterSubsystem shooterSubsystem, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter   = hw.sorter;
        this.pusher_R = hw.pusher_R;
        this.pusher_L = hw.pusher_L;
        this.shooterSubsystem = shooterSubsystem;
        this.opMode   = opMode;
        this.telemetry = telemetry;
        this.reinitPattern(pattern);
    }

    public double getFirstSorterPos() { return sorterPositions[0]; }

    public double getServoPos() { return sorter.getPosition(); }

    public Artifact[] getSorterList()        { return sorterList; }
    public void setSorterList(Artifact[] newSorterList) {
        artifactCount[0] = 0;
        for (int i = 0; i < 3; i++) {
            sorterList[i] = newSorterList[i];
            if (!newSorterList[i].getColour().equals("None")) {
                artifactCount[0]++;
            }
        }
    }
    public int getArtifactCount()            { return artifactCount[0]; }
    public void setArtifactCount(int count)  { artifactCount[0] = count; }
    public int getSorterPos()                { return curSorterPositionIndex; }

    public void reinitPattern(String pattern) {
        this.pattern = new ArrayList<>();
        for (char p : pattern.toCharArray()) this.pattern.add(p);
    }

    /** Advances sorter one slot forward (CW). Wraps 2→0. Blocked while pusher is up. */
    public void manualSpin() {
        if (isPusherUp) return;
        curSorterPositionIndex++;
        if (curSorterPositionIndex >= 3) curSorterPositionIndex = 0;
        this.sorter.setPosition(sorterPositions[curSorterPositionIndex]);
    }

    /**
     * Steps sorter one slot backward (CCW). Clamps at 0 — does not wrap.
     * Used for post-shot reload and manual back-button correction.
     */
    public void manualSpinReverse() {
        if (isPusherUp) return;
        if (curSorterPositionIndex > 0) {
            curSorterPositionIndex--;
            this.sorter.setPosition(sorterPositions[curSorterPositionIndex]);
        }
    }

    public void removeCurrentBall() {
        sorterList[curSorterPositionIndex] = new Artifact("none");
        artifactCount[0]--;
    }

    // ---------------- QUICKFIRE ----------------

    public QuickfireState quickfireState = QuickfireState.FINISH;

    public enum QuickfireState {
        PUSH, WAIT_UP, DOWN, WAIT_DOWN, SORT, WAIT_SORT, FINISH
    }

    public void startQuickfire() {
        if (selectedColour == SelectedColour.ANY)
            quickfireState = QuickfireState.PUSH;
        else
            quickfireState = QuickfireState.SORT;
    }

    public boolean isActive() {
        return quickfireState != QuickfireState.FINISH;
    }

    public void stopQuickfire() {
        quickfireState = QuickfireState.FINISH;
        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
    }

    public void quickfireState() {
        if (selectedColour == SelectedColour.ANY) {
            switch (quickfireState) {
                case PUSH:
                    if (sorterList[curSorterPositionIndex].getColour().equals("none")) {
                        quickfireState = QuickfireState.SORT;
                    } else {
                        pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                        pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                        pusherTimer.reset();
                        removeCurrentBall();
                        quickfireState = QuickfireState.WAIT_UP;
                    }
                    break;

                case WAIT_UP:
                    if (pusherTimer.milliseconds() >= 300) quickfireState = QuickfireState.DOWN;
                    break;

                case DOWN:
                    pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                    pusherTimer.reset();
                    quickfireState = QuickfireState.WAIT_DOWN;
                    break;

                case WAIT_DOWN:
                    if (pusherTimer.milliseconds() >= 400) quickfireState = QuickfireState.SORT;
                    break;

                case SORT:
                    // find next non-empty slot instead of blindly spinning
                    boolean found = false;
                    for (int i = 0; i < MAX_NUM_BALLS; i++) {
                        manualSpin();
                        if (!sorterList[curSorterPositionIndex].getColour().equals("none")) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        quickfireState = QuickfireState.FINISH;
                        break;
                    }
                    sorterTimer.reset();
                    quickfireState = QuickfireState.WAIT_SORT;
                    break;

                case WAIT_SORT:
                    if (sorterTimer.milliseconds() >= 300 && shooterSubsystem.isRPMReached())
                        quickfireState = QuickfireState.PUSH;
                    break;

                case FINISH:
                    break;
            }
        } else {
            switch (quickfireState) {
                case PUSH:
                if (sorterList[curSorterPositionIndex].getColour().equals("none")) {
                    quickfireState = QuickfireState.SORT; // slot empty, keep looking
                } else {
                    pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                    pusherTimer.reset();
                    removeCurrentBall(); // ← fix Bug 1
                    quickfireState = QuickfireState.WAIT_UP;
                }
                break;

                case WAIT_UP:
                    if (pusherTimer.milliseconds() >= 300) quickfireState = QuickfireState.DOWN;
                    break;

                case DOWN:
                    pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                    pusherTimer.reset();
                    quickfireState = QuickfireState.WAIT_DOWN;
                    break;

                case WAIT_DOWN:
                    if (pusherTimer.milliseconds() >= 400) quickfireState = QuickfireState.SORT; // ← fix Bug 4
                    break;

                case SORT:
                    String targetColour = (selectedColour == SelectedColour.GREEN) ? "Green" : "Purple";
                    boolean ballFound = false;
                    for (int i = 0; i < 3; i++) {
                        if (sorterList[i].getColour().equals(targetColour)) {
                            curSorterPositionIndex = i;
                            ballFound = true;
                            break;
                        }
                    }
                    if (ballFound) {
                        this.sorter.setPosition(sorterPositions[curSorterPositionIndex]);
                        sorterTimer.reset();
                        quickfireState = QuickfireState.WAIT_SORT; // ← fix Bug 2, always wait for RPM
                    } else {
                        quickfireState = QuickfireState.FINISH;
                    }
                    break;

                case WAIT_SORT:
                    if (shooterSubsystem.isRPMReached()) quickfireState = QuickfireState.PUSH;
                    break;

                case FINISH:
                    break;
            }
        }
    }

} //🦄🦄