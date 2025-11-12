package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;
    private final ArrayList<Artifact> sorterList;

    private ElapsedTime pushTime = new ElapsedTime();

    private ElapsedTime outtakeTime = new ElapsedTime();

    private boolean isPusherUp = false; // pusher is down

    private int curSorterPositionIndex = 0;
    private final double[] sorterPositions = new double[]{0, 0.43, 0.875};
    private int numIntakeBalls = 0;
    private long lastPushTime;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.sorterList = new ArrayList<>();

        this.reinitPattern(pattern);
    }

    public void detectColor(){

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
            telemetry.addLine("Max balls");
            telemetry.update();
            return;
        }
        turnToIntake(); // First turn to a position that allows robot to take in ball without being blocked.
        numIntakeBalls++;

        this.sorterList.add(new Artifact(color, sorter.getPosition()));
    }

    public void turnToIntake() { // turn sorter before intaking a ball

        if (sorter.getPosition() != 1 && !isPusherUp) { // ensure the sorter cannot turn more than max
            if (curSorterPositionIndex >= MAX_NUM_BALLS) {
                curSorterPositionIndex = 0;
            }
            sorter.setPosition(this.sorterPositions[curSorterPositionIndex]);
            curSorterPositionIndex++;
        }
    }
    public void push(){
        telemetry.addLine("Pusher up");
//      pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
        pushTime.reset();
        isPusherUp = true;

        if (isPusherUp && (pushTime.seconds() >= 2)) {
            telemetry.addLine("Pusher down");
            //pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
            isPusherUp = false;
        }
        telemetry.update();
    }

    // quickFire fires a random ball
    public void quickFire() {
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