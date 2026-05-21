package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.ArrayList;

public class SorterSubsystem {

    // ---------------- SUBSYSTEMS ----------------
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private ShooterSubsystem shooterSubsystem;

    // ---------------- HARDWARE ----------------
    private final Servo sorter;
    private Servo pusher_L;
    private Servo pusher_R;
    private Servo gate;
    private Servo light;

    // ---------------- TIMERS ----------------
    private static final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    // ---------------- COLOUR SELECTION ----------------

    public SelectedColour selectedColour = SelectedColour.ANY;

    public enum SelectedColour {
        ANY,
        GREEN,
        PURPLE
    }

    // ---------------- OTHER ----------------
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final Artifact[] sorterList = new Artifact[]{ new Artifact("none"), new Artifact("none"), new Artifact("none") };;
    private int artifactCount[] = new int[]{ 0 };
    private double sorterPosition = 0.0;
    private boolean isPusherUp = false;
    public static final int MAX_NUM_BALLS = 3;

    private int curSorterPositionIndex = 0;
    private final double[] sorterPositions = new double[]{0.00, 0.42, 0.875};

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.pusher_R = hw.pusher_R;
        this.pusher_L = hw.pusher_L;
        this.opMode = opMode;
        this.telemetry = telemetry;

        this.reinitPattern(pattern);
    }

    // get sorter list used for colour sensor subsystem
    public Artifact[] getSorterList() { return sorterList; }
    public int getArtifactCount() { return artifactCount[0]; }
    public void setArtifactCount(int count) { artifactCount[0] = count; }
    // get sorter position
    public int getSorterPos() { return curSorterPositionIndex; }

    // reinitPattern re-initializes the pattern. Call reinitPattern when reading a new pattern.
    public void reinitPattern(String pattern) {
        this.pattern = new ArrayList<>();
        for (char p : pattern.toCharArray()) {
            this.pattern.add(p);
        }
    }

    public void manualSpin() {
        if (isPusherUp) {
            return;
        }
        curSorterPositionIndex++;

        if (curSorterPositionIndex >= 3) {
            curSorterPositionIndex = 0;
        }
        this.sorter.setPosition(sorterPositions[curSorterPositionIndex]);
    }

    // ---------------- QUICKFIRE ----------------

    public QuickfireState quickfireState = QuickfireState.FINISH;
    public enum QuickfireState {
        PUSH,
        WAIT_UP,
        DOWN,
        WAIT_DOWN,
        SORT,
        WAIT_SORT,
        FINISH
    }
    public void startQuickfire() {
        // add changing first quickfire position based on sorterList
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
                        sorterList[curSorterPositionIndex] = new Artifact("none");
                        artifactCount[0]--;
                        quickfireState = QuickfireState.WAIT_UP;
                    }
                    break;

                case WAIT_UP:
                    if (pusherTimer.milliseconds() >= 300) {
                        quickfireState = QuickfireState.DOWN;
                    }
                    break;

                case DOWN:
                    pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                    pusherTimer.reset();
                    quickfireState = QuickfireState.WAIT_DOWN;
                    break;

                case WAIT_DOWN:
                    if (pusherTimer.milliseconds() >= 400) {
                        quickfireState = QuickfireState.SORT;
                    }
                    break;

                case SORT:
                    boolean allEmpty = true;
                    for (int i = 0; i < MAX_NUM_BALLS; i++) {
                        if (!sorterList[i].getColour().equals("none")) {
                            allEmpty = true;
                            break;
                        }
                    }
                    if (allEmpty) {
                        quickfireState = QuickfireState.FINISH;
                        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                        break;
                    }
                    manualSpin();
                    sorterTimer.reset();
                    quickfireState = QuickfireState.WAIT_SORT;
                    break;

                case WAIT_SORT:
                    if (sorterTimer.milliseconds() >= 470) {
                        quickfireState = QuickfireState.PUSH;
                    }
                    break;

                case FINISH:
                    break;
            }
        } else {
            switch (quickfireState) {

                case PUSH:
                    pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                    pusherTimer.reset();
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
                    pusherTimer.reset();
                    quickfireState = QuickfireState.WAIT_DOWN;
                    break;

                case WAIT_DOWN:
                    if (pusherTimer.milliseconds() >= 400) {
                        quickfireState = QuickfireState.FINISH;
                    }
                    break;

                case SORT:
                    boolean ballFound;
                    switch (selectedColour) {
                        case GREEN:
                            ballFound = false;

                            for (int i = 0; i < 3; i++) {
                                if (sorterList[i].getColour().equals("Green")) {
                                    curSorterPositionIndex = i;
                                    ballFound = true;
                                    break;
                                }
                            }

                            if (ballFound) {
                                this.sorter.setPosition(sorterPositions[curSorterPositionIndex]);
                                quickfireState = QuickfireState.PUSH;
                            } else {
                                quickfireState = QuickfireState.FINISH;
                            }
                            break;
                        case PURPLE:
                            ballFound = false;

                            for (int i = 0; i < 3; i++) {
                                if (sorterList[i].getColour().equals("Purple")) {
                                    curSorterPositionIndex = i;
                                    ballFound = true;
                                    break;
                                }
                            }

                            if (ballFound) {
                                this.sorter.setPosition(sorterPositions[curSorterPositionIndex]);
                                sorterList[curSorterPositionIndex] = new Artifact("none");
                                artifactCount[0]--;
                                sorterTimer.reset();
                                quickfireState = QuickfireState.WAIT_SORT;
                                break;
                            } else {
                                quickfireState = QuickfireState.FINISH;
                            }
                            break;
                    }
                    break;

                    case WAIT_SORT:
                        if (sorterTimer.milliseconds() >= 470) {
                            quickfireState = QuickfireState.PUSH;
                        }
                        break;

                    case FINISH:
                        break;
            }
        }
    }


} //🦄🦄