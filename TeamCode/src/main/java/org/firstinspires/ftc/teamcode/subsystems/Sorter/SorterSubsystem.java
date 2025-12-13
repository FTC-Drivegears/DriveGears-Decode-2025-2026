package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.ArrayList;


public class SorterSubsystem {
    private Hardware hw;
    public static final int MAX_NUM_BALLS = 3;
    private final Servo sorter;
    private final Servo pusher;
    private static final double PUSHER_UP = 0.0;
    private static final double PUSHER_DOWN = 1.0;
    private static final long PUSHER_TIME = 150;

    // Color sensors do not work. One reports 0 values. Another one reported 646 for red, 2000 for other numbers.
//    public final ColorSensor colourSensor;
//    public final ColorSensor colourSensor2;

    public final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final ArrayList<Artifact> sorterList;

    private final ElapsedTime sorterSpinTime = new ElapsedTime();

//    private final ElapsedTime pusherTimer = new ElapsedTime();

    private final ElapsedTime pusherTimer = new ElapsedTime();

    private final ElapsedTime spinForIntakeTime = new ElapsedTime();
    private final ElapsedTime spinForOuttakeTime = new ElapsedTime();
    private boolean isPusherUp = false; // pusher is down
    private int curSorterPositionIndex = 0;

    private final double[] sorterPositions = new double[]{0.085, 0.515, 0.96};
    private final double sorterPosition1 = 0.085;
    private final double sorterPosition2 = 0.515;
    private final double sorterPosition3 = 0.96;



    public NormalizedColorSensor colourSensor1;
    public NormalizedColorSensor colourSensor2;

    public boolean isPurple = false;

    public boolean isGreen = false;

    private boolean isBall = false;

    private static final long SORTER_TIME = 400;

//    public TransferState state = TransferState.FIRST;

//    public enum TransferState {
//        PUSH_UP,
//        PUSH_DOWN,
//        SORT,
//        FIRST
//    }

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
        this.colourSensor1 = hw.colourSensor1;
        this.colourSensor2 = hw.colourSensor2;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.sorterList = new ArrayList<>(3);
        sorterList.add(null);
        sorterList.add(null);
        sorterList.add(null);

        this.reinitPattern(pattern);
    }

    // reinitPattern re-initializes the pattern. Call reinitPattern when reading a new pattern.
    public void reinitPattern(String pattern) {
        this.pattern = new ArrayList<>();
        for (char p : pattern.toCharArray()) {
            this.pattern.add(p);
        }
    }

    public void setIsPusherUp(boolean isPusherUp) {
        this.isPusherUp = isPusherUp;
    }

    public boolean getIsPusherUp() {
        return this.isPusherUp;
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

    public void intakeBall(char color) {
        // fail-safe
        if (this.sorterList.size() == MAX_NUM_BALLS) {
            telemetry.addLine("Cannot intake any more balls, max capacity");
            return;
        }

        this.turnToIntake(color); // First turn to a position that allows robot to take in ball without being blocked

    }

    public void turnToIntake(char color) { // turn sorter before intaking a ball
        if (!isPusherUp && sorter.getPosition() != 1) { // ensure the sorter cannot turn more than max
            if (curSorterPositionIndex >= MAX_NUM_BALLS) {
                curSorterPositionIndex = 0;
            }

            telemetry.addLine("I am pressing dpad left yes");
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
            sorterSpinTime.reset();
            sorterList.add(new Artifact(color, sorter.getPosition()));
            spinForIntakeTime.reset();
            curSorterPositionIndex++;
        } else {
            telemetry.addLine("pusher is up, CANNOT turn sorter");
        }
    }

    public void turnToNextPos() {
        if (curSorterPositionIndex < MAX_NUM_BALLS) {
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
            curSorterPositionIndex++;
        }
    }

    public void outtakeToNextPos() {
        if (curSorterPositionIndex > 0) {
            curSorterPositionIndex--;
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
        }
    }

    public int getCurSorterPositionIndex() {
        return curSorterPositionIndex;
    }

    public void turnForIntake() { // turn sorter before intaking a ball
        if (!isPusherUp && sorter.getPosition() != 1) { // ensure the sorter cannot turn more than max
            if (curSorterPositionIndex >= MAX_NUM_BALLS) {
                curSorterPositionIndex = 0;
            }

            telemetry.addLine("I am pressing dpad left yes");
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
            sorterSpinTime.reset();
            spinForIntakeTime.reset();
            curSorterPositionIndex++;
        } else {
            telemetry.addLine("pusher is up, CANNOT turn sorter");
        }
    }

    public void detectColor() {
        NormalizedRGBA colours1 = colourSensor1.getNormalizedColors();
        NormalizedRGBA colours2 = colourSensor2.getNormalizedColors();

        float hue = JavaUtil.colorToHue(colours1.toColor());
        float hue2 = JavaUtil.colorToHue(colours2.toColor());

        if (hue < 163 && hue > 150 || hue2 < 163 && hue2 > 150) {
            telemetry.addData("Color", "Green");
            isGreen = true;

            if (sorterList.get(curSorterPositionIndex) == null) {
                sorterList.set(curSorterPositionIndex, new Artifact('g', sorterPositions[curSorterPositionIndex]));
            }
        } else if (hue < 350 && hue > 165 || hue2 < 350 && hue2 > 165) {
            telemetry.addData("Color", "Purple");
            isPurple = true;
            if (sorterList.get(curSorterPositionIndex) == null) {
                sorterList.set(curSorterPositionIndex, new Artifact('p', sorterPositions[curSorterPositionIndex]));
            }
        } else {
            telemetry.addLine("Nothing is here");
        }

        if (isGreen || isPurple) {
            isBall = true;
        } else
            isBall = false;
    }

    // push will wait 750ms to push up, then wait for less time to go back down.
//    public void push() {
//        if (sorterSpinTime.milliseconds() >= 1500) {
//            pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
//            telemetry.addLine("Pusher up");
//            isPusherUp = true;
//            pushTime.reset();
//        }
//    }

//    public void pushDown() {
//        if (isPusherUp && pushTime.milliseconds() >= 1500) {
//            pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
//            telemetry.addLine("Pusher down");
//            isPusherUp = false;
//        }
//    }

    // quickFire fires a random ball
//    public void quickFire() {
//        if (this.sorterList.isEmpty()){
//            return;
//        }
//
//        this.waitForSpinAndPush(0);
//    }

    public boolean getIsBall() {
        return isBall;
    }

    public void setIsBall(boolean value) {
        isBall = value;
    }

    public void outtakeBall(char colorToRemove) {
        if (this.sorterList.isEmpty()) {
            telemetry.addLine("No ball in sorter");
            return;
        }

        telemetry.addData("color to remove", colorToRemove);
        this.findBallIndexAndSpin(colorToRemove);
    }


    // outtakeBallWithPattern only works with pattern read from tag.
    // However reading from tag means we must execute the steps right away.
    // Otherwise pattern will be continously read. There is no good stopping point in teleop.
//    public void outtakeBallWithPattern() {
//        if (this.pattern.isEmpty()){
//            telemetry.addLine("Pattern is empty");
//            telemetry.update();
//            return;
//        }
//        if (this.sorterList.isEmpty()) {
//            telemetry.addLine("No ball in sorter");
//            telemetry.update();
//            return;
//        }
//
//        telemetry.addData("current pattern", this.pattern);
//        char colorToRemove = this.pattern.get(0);
//        telemetry.addData("color to remove", colorToRemove);
//
//        this.spinToBallAndPush(colorToRemove);
//        this.pattern.remove(0);
//    }

    private void findBallIndexAndSpin(char colorToRemove) { // for outtake spinning
        int ballIndexToRemoveFromSorter = -1;
        telemetry.addData("num balls left", this.sorterList.size());
        for (int i = 0; i < this.sorterList.size(); i++) {
            if (this.sorterList.get(i) != null && this.sorterList.get(i).getColor() == colorToRemove) {
                ballIndexToRemoveFromSorter = i;
                break;
            }
        }

        if (ballIndexToRemoveFromSorter == -1) {
            telemetry.addData("color not found: ", colorToRemove);
            telemetry.update();
            return;
        }

        this.sorterSpinToOuttake(ballIndexToRemoveFromSorter);
    }

    private void sorterSpinToOuttake(int ballIndexToRemoveFromSorter) { //
        if (ballIndexToRemoveFromSorter >= this.sorterList.size()) {
            return;
        }

        if (!isPusherUp) {
            sorter.setPosition(this.sorterList.get(ballIndexToRemoveFromSorter).getPosition());
            curSorterPositionIndex = ballIndexToRemoveFromSorter;
            telemetry.addLine("index" + ballIndexToRemoveFromSorter);
        } else {
            telemetry.addLine("pusher is up, CANNOT turn sorter");
        }

        this.sorterList.set(ballIndexToRemoveFromSorter, null);
    }
}

//    public boolean transfer() {
//        switch (state) {
//            case FIRST:
//                pusherUp();
//                pusherTimer.reset();
//                state = TransferState.PUSH_UP;
//                return true;
//
//            case PUSH_UP:
//                if (pusherTimer.milliseconds() >= PUSHER_TIME) {
//                    pusherDown();
//                    sorterSpinTime.reset();
//                    state = TransferState.PUSH_DOWN;
//                }
//                return true;
//
//            case PUSH_DOWN:
//                if (sorterSpinTime.milliseconds() >= SORTER_TIME) {
//                    state = TransferState.SORT;
//                }
//                return true;
//
//            case SORT:
//                curSorterPositionIndex = (curSorterPositionIndex + 1) % 3;
//                if (curSorterPositionIndex == 0) {
//                    hw.sorter.setPosition(sorterPosition1);
//                }
//                if (curSorterPositionIndex == 1) {
//                    hw.sorter.setPosition(sorterPosition2);
//                }
//                if (curSorterPositionIndex == 2) {
//                    hw.sorter.setPosition(sorterPosition3);
//                }
//
//                state = TransferState.FIRST;
//                return false;
//
//            default:
//                return false;
//        }
//    }
//    public void pusherUp() {
//        hw.pusher.setPosition(PUSHER_UP);
//    }
//
//    public void pusherDown() {
//        hw.pusher.setPosition(PUSHER_DOWN);
//    }
//}