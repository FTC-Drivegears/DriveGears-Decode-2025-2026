package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

import java.util.ArrayList;

public class SorterSubsystem {
    public static final int MAX_NUM_BALLS = 3;
    private final Servo sorter;
    private final Servo pusher;
    public final ColorSensor colourSensor;
    public final ColorSensor colourSensor2;

    public final Telemetry telemetry;
    private final Servo pusher;

    // Color sensors do not work. One reports 0 values. Another one reported 646 for red, 2000 for other numbers.
//    public final ColorSensor colourSensor;
//    public final ColorSensor colourSensor2;

    public final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final ArrayList<Artifact> sorterList;

    private final ElapsedTime pushTime = new ElapsedTime();
    private final ElapsedTime pushTime = new ElapsedTime();
    private final ElapsedTime sorterSpinTime = new ElapsedTime();

    private final ElapsedTime spinForIntakeTime = new ElapsedTime();
    private final ElapsedTime spinForOuttakeTime = new ElapsedTime();
    private final ElapsedTime detectColorTime = new ElapsedTime();
    private boolean isPusherUp = false; // pusher is down
    private int curSorterPositionIndex = 0;
    private final double[] sorterPositions = new double[]{0.1, 0.43, 0.875};

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
        this.colourSensor = hw.colourSensor;
        this.colourSensor2 = hw.colourSensor2;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.sorterList = new ArrayList<>();

        this.reinitPattern(pattern);
    }

    // reinitPattern re-initializes the pattern. Call reinitPattern when reading a new pattern.
    public void reinitPattern(String pattern) {
        this.pattern = new ArrayList<>();
        for (char p: pattern.toCharArray()) {
            this.pattern.add(p);
        }
    }

    public void setIsPusherUp(boolean isPusherUp) {
        this.isPusherUp = isPusherUp;
    }

    public void intakeBall(char color) {
        // fail-safe
        if (this.sorterList.size() == MAX_NUM_BALLS){
            telemetry.addLine("Cannot intake any more balls, max capacity");
            telemetry.update();
            return;
        }

//        telemetry.addData("numIntakeBalls before turn to intake", numIntakeBalls);
        turnToIntake(); // First turn to a position that allows robot to take in ball without being blocked.
        numIntakeBalls++;
        telemetry.update();

        this.sorterList.add(new Artifact(color, sorter.getPosition()));
    }

    public void turnToIntake() { // turn sorter before intaking a ball

        if (sorter.getPosition() != 1 && !isPusherUp) { // ensure the sorter cannot turn more than max
            telemetry.addData("how many balls?", this.sorterList.size());
            if (curSorterPositionIndex >= MAX_NUM_BALLS) {
                curSorterPositionIndex = 0;
            }
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
            curSorterPositionIndex++;
            telemetry.addLine("turning to position");
            telemetry.update();
        }
        telemetry.update();
    }

    boolean isGreen(int red, int green, int blue, int alpha) {
        return red < 55 && red > 40 && green < 110 && green > 90 && blue < 90 && blue > 70 && alpha < 85 && alpha > 65;
    }

    // push will wait 750ms to push up, then wait for less time to go back down.
    public void push() {
        if (sorterSpinTime.milliseconds() >= 1500) {
            pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
            telemetry.addLine("Pusher up");
            isPusherUp = true;
            pushTime.reset();
        }
    }

    public void pushDown() {
        if (isPusherUp && pushTime.milliseconds() >= 350) {
            pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
            telemetry.addLine("Pusher down");
            isPusherUp = false;
        }
    }

    // quickFire fires a random ball
    public void quickFire() {
        if (this.sorterList.isEmpty()){
            return;
        }
        if (!isPusherUp){
            sorter.setPosition(this.sorterList.get(0).getPosition());
            telemetry.addData("Pushing out this ball", this.sorterList.get(0));
            telemetry.update();
        }

        this.sorterList.remove(0);
    }

    // Test outtakeBall after quickFire
    // outtakeBall fires a ball from pattern
//    public void outtakeBall() {
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
//        telemetry.update();
//
//        int ballIndexToRemoveFromSorter = -1;
//        telemetry.addData("num balls left", this.sorterList.size());
//        for (int i = 0; i < this.sorterList.size(); i++){
//            if (this.sorterList.get(i).getColor() == colorToRemove){
//                ballIndexToRemoveFromSorter = i;
//                break;
//            }
//        }
//        telemetry.update();
//
//        if (ballIndexToRemoveFromSorter == -1){
//            telemetry.addData("color not found: ", colorToRemove);
//            telemetry.update();
//            return;
//        }
//
//        for(int i = 0; i <= MAX_NUM_BALLS; i++){
//            if (!isPusherUp) {
//                sorter.setPosition(this.sorterList.get(ballIndexToRemoveFromSorter).getPosition());
//                telemetry.addLine("sorter moving" + i);
//                push();
//                telemetry.addLine("pusher moved" + i);
//                telemetry.update();
//            }
//        }
//
//        this.sorterList.remove(ballIndexToRemoveFromSorter);
//        this.pattern.remove(0);
//    }
}