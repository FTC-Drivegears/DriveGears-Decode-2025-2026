package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

import java.util.ArrayList;

public class SorterSubsystem {
    public static final int MAX_NUM_BALLS = 3;
    private final Servo sorter;
    private final ColorSensor colourSensor1;
    private final ColorSensor colourSensor2;
    private Servo pusher;
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final ArrayList<Artifact> sorterList;

    boolean detectedColour = false;

    public int red1;
    public int blue1;
    public int green1;
    public int alpha1;

    public int red2;
    public int blue2;
    public int green2;
    public int alpha2;

    private long lastColourDetectionTime;

    private ElapsedTime pushTime = new ElapsedTime();

    private boolean isPusherUp = false;

    private int curSorterPositionIndex = 0;
    private final double[] sorterPositions = new double[]{0.085, 0.515, 0.96};
    private int numIntakeBalls = 0;
    private long lastPushTime;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.colourSensor1 = hw.colour1;
        this.colourSensor2 = hw.colour2;
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

    public void intakeBall(char color){
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

    public void turnToIntake() { // turn sorter before intake a ball

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

    public void turnSorter() {
        // If the sorterList is full it stops
        if (sorterList.size() == MAX_NUM_BALLS) {
            return;
        }
        if (sorter.getPosition() == sorterPositions[1]) {
            sorter.setPosition(sorterPositions[2]);
        }
        if (sorter.getPosition() == sorterPositions[0]) {
            sorter.setPosition(sorterPositions[1]);
        }
    }

    public void detectColour() {
        red1 = colourSensor1.red();
        green1 = colourSensor1.green();
        blue1 = colourSensor1.blue();
        alpha1 = colourSensor1.alpha();

        red2 = colourSensor2.red();
        green2 = colourSensor2.green();
        blue2 = colourSensor2.blue();
        alpha2 = colourSensor2.alpha();

        // If the sorter is full it stops
        if (sorterList.size() == MAX_NUM_BALLS) {
            return;
        }

        // Purple ball is detected
        if (red1 > 50 && red1 < 65 && green1 < 95 && green1 > 80 && blue1 < 95 && blue1 > 78 && alpha1 < 85 && alpha1 > 70) {
            telemetry.addLine("Purple Detected");
            telemetry.update();
            if (!detectedColour) {
                detectedColour = true;
                sorterList.add(new Artifact('P', sorter.getPosition()));
                turnSorter();
            }
        }

        // Green ball is detected
        else if (red1 < 55 && red1 > 40 && green1 < 110 && green1 > 90 && blue1 < 90 && blue1 > 70 && alpha1 < 85 && alpha1 > 65) {
            telemetry.addLine("Green Detected");
            telemetry.update();
            if (!detectedColour) {
                detectedColour = true;
                sorterList.add(new Artifact('G', sorter.getPosition()));
                turnSorter();
            }
        }

        else {
            if (System.currentTimeMillis() - lastColourDetectionTime > 300) {
                detectedColour = false;
            }
        }

        if (detectedColour) {
            lastColourDetectionTime = System.currentTimeMillis();
        }
    }
    public void push(){

        pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
        pushTime.reset();
        isPusherUp = true;

        if (isPusherUp && pushTime.milliseconds() >= 500) {
            pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
            isPusherUp = false;
        }
    }

    // quickFire fires a random ball
    public void quickFire() {
        if (this.sorterList.isEmpty()){
            telemetry.addLine("Nothing in sorter");
            telemetry.update();
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