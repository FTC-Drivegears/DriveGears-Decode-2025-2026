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
    private Servo pusher;
    private ColorSensor colorSensor;
    private ColorSensor colorSensor2;
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final ArrayList<Artifact> sorterList;

    private ElapsedTime pushTime = new ElapsedTime();
    private ElapsedTime outtakeTime = new ElapsedTime();
    private boolean isPusherUp = false; // pusher is down
    private int curSorterPositionIndex = 0;
    private final double[] sorterPositions = new double[]{0.0, 0.42, 0.875};
    private int numIntakeBalls = 0;
    private long lastPushTime;

    private boolean purpleArt;
    private boolean greenArt;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.pusher = hw.pusher;
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

    public void intakeBall(char color){
        // fail-safe
        if (this.sorterList.size() == MAX_NUM_BALLS){
            telemetry.addLine("Cannot intake any more balls, max capacity");
            telemetry.update();
            return;
        }

        turnToIntake(); // First turn to a position that allows robot to take in ball without being blocked
        numIntakeBalls++;
        sorterList.add(new Artifact(color, sorter.getPosition()));
        // TODO
        //detectColor(); <-- add this later when two color sensors are on
    }

    public void turnToIntake() { // turn sorter before intaking a ball

        if (sorter.getPosition() != 1 && !isPusherUp) { // ensure the sorter cannot turn more than max
            if (curSorterPositionIndex >= MAX_NUM_BALLS) {
                curSorterPositionIndex = 0;
            }
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
            curSorterPositionIndex++;
            telemetry.addLine("turning to position");
        }
        telemetry.update();
    }
    public void push(){
        pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
        pushTime.reset();
        isPusherUp = true;

        if (pushTime.milliseconds() >= 2) {
            pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
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

    public void detectColor() {
        int red = colorSensor.red();
        int red2 = colorSensor2.red();

        int green = colorSensor.green();
        int green2 = colorSensor2.green();

        int blue = colorSensor.blue();
        int blue2 = colorSensor2.blue();

        int alpha = colorSensor.alpha();
        int alpha2 = colorSensor2.alpha();

        // Purple ball is detected
        if (isPurple(red, green, blue, alpha) && isPurple(red2, green2, blue2, alpha2)) {
            telemetry.addLine("Purple Detected");
            telemetry.update();
            sorterList.add(new Artifact('p', sorter.getPosition()));
        } else if (isGreen(red, green, blue, alpha) && isGreen(red2, green2, blue2, alpha2)) {
            telemetry.addLine("Green Detected");
            telemetry.update();
            sorterList.add(new Artifact('g', sorter.getPosition()));
        }
    }

    // quickFire fires a random ball
    public void quickFire(){
        outtakeTime.reset();
        pushTime.reset();

        if (this.sorterList.isEmpty()){
            return;
        }

        if (!isPusherUp) { // means pusher is down
            if (outtakeTime.seconds() >= 1){
                sorter.setPosition(this.sorterList.get(0).getPosition());
                telemetry.update();
                this.sorterList.remove(0);
            }
        }
        push();
    }

    public void outtakeBall() {
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

        for(int i = 0; i <= MAX_NUM_BALLS; i++){
            if (!isPusherUp) {
                sorter.setPosition(this.sorterList.get(ballIndexToRemoveFromSorter).getPosition());
                telemetry.addLine("sorter moving" + i);
                // might need to delay to wait for sorter to spin before calling push
//                push();
                telemetry.addLine("pusher moved" + i);
                telemetry.update();
            }
        }

        this.sorterList.remove(ballIndexToRemoveFromSorter);
        this.pattern.remove(0);
    }
}