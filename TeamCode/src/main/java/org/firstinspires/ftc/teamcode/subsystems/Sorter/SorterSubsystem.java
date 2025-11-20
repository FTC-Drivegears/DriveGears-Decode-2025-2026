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
//    private final ColorSensor colourSensor;
//    private final ColorSensor colourSensor2;
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final ArrayList<Artifact> sorterList;

    private final ElapsedTime pushTime = new ElapsedTime();
    private final ElapsedTime outtakeTime = new ElapsedTime();
    private boolean isPusherUp = false; // pusher is down
    private int curSorterPositionIndex = 0;
    private final double[] sorterPositions = new double[]{0.0, 0.43, 0.875};

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
//        this.colourSensor = hw.colourSensor;
//        this.colourSensor2 = hw.colourSensor2;
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

    public boolean isMaxBallsReached() {
        return this.sorterList.size() == MAX_NUM_BALLS;
    }

    public void intakeBall(char color) {
        // fail-safe
        if (this.sorterList.size() == MAX_NUM_BALLS){
            telemetry.addLine("Cannot intake any more balls, max capacity");
            telemetry.update();
            return;
        }

        turnToIntake(); // First turn to a position that allows robot to take in ball without being blocked

        // If color sensor is not on use this code:
        sorterList.add(new Artifact(color, sorter.getPosition()));

        // TODO: If color sensor is on, use this following block instead, remove char color in input from intakeBall parameter
        //detectColor(); <-- add this later when two color sensors are on
    }

    public void turnToIntake() { // turn sorter before intaking a ball

        if (sorter.getPosition() != 1 && !isPusherUp) { // ensure the sorter cannot turn more than max
            if (curSorterPositionIndex >= MAX_NUM_BALLS) {
                curSorterPositionIndex = 0;
            }
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
            curSorterPositionIndex++;
        }
        telemetry.update();
    }

    // push will wait 750ms to push up, then wait for less time to go back down.
    public void push() {
        if (pushTime.milliseconds() >= 700) { // Wait for plate to spin.
            pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
            pushTime.reset();
            telemetry.addLine("Pusher up");
            isPusherUp = true;
        }

        if (pushTime.milliseconds() >= 250) {
            pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
            pushTime.reset();
            telemetry.addLine("Pusher down");
            isPusherUp = false;
        }
        telemetry.update();
    }

    boolean isPurple(int red, int green, int blue, int alpha) {
        return red > 50 && red < 65 && green < 95 && green > 80 && blue < 95 && blue > 78 && alpha < 85 && alpha > 70;
    }

    boolean isGreen(int red, int green, int blue, int alpha) {
        return red < 55 && red > 40 && green < 110 && green > 90 && blue < 90 && blue > 70 && alpha < 85 && alpha > 65;
    }

//    public void detectColor() {
//        int red = colourSensor.red();
//        int red2 = colourSensor2.red();
//
//        int green = colourSensor.green();
//        int green2 = colourSensor2.green();
//
//        int blue = colourSensor.blue();
//        int blue2 = colourSensor2.blue();
//
//        int alpha = colourSensor.alpha();
//        int alpha2 = colourSensor2.alpha();
//
//        // Purple ball is detected
//        if (isPurple(red, green, blue, alpha) && isPurple(red2, green2, blue2, alpha2)) {
//            telemetry.addLine("Purple Detected");
//            sorterList.add(new Artifact('p', sorter.getPosition()));
//        } else if (isGreen(red, green, blue, alpha) && isGreen(red2, green2, blue2, alpha2)) {
//            telemetry.addLine("Green Detected");
//            sorterList.add(new Artifact('g', sorter.getPosition()));
//        }
//
//        telemetry.update();
//    }

    // quickFire fires a random ball
    public void quickFire(){
        outtakeTime.reset();

        if (this.sorterList.isEmpty()){
            return;
        }

        if (!isPusherUp) { // means pusher is down
            if (outtakeTime.seconds() >= 1){
                sorter.setPosition(this.sorterList.get(0).getPosition());
                this.sorterList.remove(0);
            }
        }
        this.push();
    }

    public void outtakeBall(char colorToRemove) {
        outtakeTime.reset();

        if (this.sorterList.isEmpty()) {
            telemetry.addLine("No ball in sorter");
            telemetry.update();
            return;
        }

        telemetry.addData("color to remove", colorToRemove);
        this.spinToBallAndPush(colorToRemove);
    }


    // outtakeBallWithPattern only works with pattern read from tag.
    // However reading from tag means we must execute the steps right away.
    // Otherwise pattern will be continously read. There is no good stopping point in teleop.
    public void outtakeBallWithPattern() {
        outtakeTime.reset();

        if (this.pattern.isEmpty()){
            telemetry.addLine("Pattern is empty");
            telemetry.update();
            return;
        }
        if (this.sorterList.isEmpty()) {
            telemetry.addLine("No ball in sorter");
            telemetry.update();
            return;
        }

        telemetry.addData("current pattern", this.pattern);
        char colorToRemove = this.pattern.get(0);
        telemetry.addData("color to remove", colorToRemove);

        this.spinToBallAndPush(colorToRemove);
        this.pattern.remove(0);
    }

    private void spinToBallAndPush(char colorToRemove) {
        int ballIndexToRemoveFromSorter = -1;
        telemetry.addData("num balls left", this.sorterList.size());
        for (int i = 0; i < this.sorterList.size(); i++){
            if (this.sorterList.get(i).getColor() == colorToRemove){
                ballIndexToRemoveFromSorter = i;
                break;
            }
        }

        if (ballIndexToRemoveFromSorter == -1){
            telemetry.addData("color not found: ", colorToRemove);
            telemetry.update();
            return;
        }

        if (!isPusherUp) {
            if (outtakeTime.seconds() >= 1) {
                sorter.setPosition(this.sorterList.get(ballIndexToRemoveFromSorter).getPosition());
                telemetry.addLine("index" + ballIndexToRemoveFromSorter);
            }
        } else {
            telemetry.addLine("pusher is up, CANNOT turn sorter");
        }
        this.push();

        this.sorterList.remove(ballIndexToRemoveFromSorter);
    }
}