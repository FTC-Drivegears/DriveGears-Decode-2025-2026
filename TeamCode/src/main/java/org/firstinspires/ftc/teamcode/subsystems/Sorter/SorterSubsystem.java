package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.opmodes.tests.autoOp.NewBlueAutoOp;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

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
    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    // ---------------- OTHER ----------------
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final ArrayList<Artifact> sorterList;
    private double sorterPosition = 0.0;
    private boolean isPusherUp = false;
    public static final int MAX_NUM_BALLS = 3;

    private int curSorterPositionIndex = 0;
    private final double[] sorterPositions = new double[]{0.0, 0.42, 0.875};
    private int numIntakeBalls = 0;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.pusher_R = hw.pusher_R;
        this.pusher_L = hw.pusher_L;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.sorterList = new ArrayList<>();

        this.reinitPattern(pattern);
    }

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
        if (curSorterPositionIndex >= 3) {
            curSorterPositionIndex = 0;
        }
        this.sorter.setPosition(sorterPositions[curSorterPositionIndex]);
        curSorterPositionIndex++;
    }

    //QUICKFIRE WHEN: CAMERA ALIGNED, RPM REACHED & GAMEPAD1.DPADLEFT CLICK AGAIN = STOP QUICKFIRE
    public QuickFire quickfireState = QuickFire.PUSH;

    public enum QuickFire {
        PUSH,
        SORT,
        PUSH_2,
        SORT_2,
        FINISH
    }

    public void quickfireState() {
//        pusher_R = hw.pusher_R;
//        pusher_L = hw.pusher_L;
//        light = hw.light;
//        gate = hw.gate;
//
//        hw.sorter.setPosition(0.0);
//        hw.light.setPosition(0.0);
//        gate.setPosition(0.5);
//
//
//        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
//        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);

        switch (quickfireState) {
            //if camera aligned, turn on & wait rpm
//            if () {
            case PUSH:
//                pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
//                pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
//                if (pusherTimer.milliseconds() > 500);{
//                pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
//                pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);

//                        pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
//                        pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
//                        pusherTimer.reset();
//                        pusherTimer.milliseconds() >= 500
//                    pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
//                    pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
//                break;


//            case SORT:
//                sorterPosition = (sorterPosition + 1) % 3;
//                sorterTimer.reset();

//                if (curSorterPositionIndex >= 3) {
//                    sorterPosition = (sorterPosition + 1) % 3;
//                    sorterTimer.reset();
//                    if (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
//                    else if (sorterPosition == 1) hw.sorter.setPosition(0.43);
//                    else hw.sorter.setPosition(0.875);
//                    manualSpin();
//                }
//                sorterPosition = (sorterPosition + 1) % 3;
//        sorterTimer.reset();
//                if (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
//                else if (sorterPosition == 1) hw.sorter.setPosition(0.43);
//                else hw.sorter.setPosition(0.875);

            case FINISH:
                break;
            }
        }
    }

//
//            if (sorterPosition == 0)
//                hw.sorter.setPosition(0.0);
//            else if (sorterPosition == 1)
//                hw.sorter.setPosition(0.43);
//            else
//                hw.sorter.setPosition(0.875);
//            break;
//    }
//}
//sorterPosition = (sorterPosition + 1) % 3;
//        sorterTimer.reset();
//                if (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
//                else if (sorterPosition == 1) hw.sorter.setPosition(0.43);
//                else hw.sorter.setPosition(0.875);