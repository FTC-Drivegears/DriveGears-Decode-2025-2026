package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.ArrayList;

public class SorterSubsystem {
    public static final int MAX_NUM_BALLS = 3;

    private static final double PUSHER_POSITION = 0;

    private final Servo sorter;

    //  private final ColorSensor colour;
    private final Telemetry telemetry;
    public final LinearOpMode opMode;
    private ArrayList<Character> pattern;

    private ArrayList<Artifact> sorterList;
    private int numIntakeBalls = 0;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
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

//    public void detectColour() {
//        Telemetry.Item detectingColor = telemetry.addData("Detecting color", sorterList.get(0));
//        telemetry.addLine(String.valueOf(detectingColor));
//        telemetry.update();
//
////        // Purple ball is detected
////        if (isPurple) {
////            telemetry.addLine("Purple Detected");
////            telemetry.update();
////            sorterList.add(new Artifact("purple", sorter.getPosition()));
////            pattern.add("purple");
////            turnSorter();
////        } else if (isGreen) {
////            telemetry.addLine("Green Detected");
////            telemetry.update();
////            sorterList.add(new Artifact("green", sorter.getPosition()));
////            pattern.add("green");
////            turnSorter();
////        } else {
////            telemetry.addLine("No color");
////            telemetry.update();
////        }
//    }

    public void intakeBall(char color){
        // fail-safe
        if (this.sorterList.size() == MAX_NUM_BALLS){
            telemetry.addLine("Cannot intake any more balls, max capacity");
            telemetry.update();
            return;
        }

        telemetry.addData("numIntakeBalls before turn to intake", numIntakeBalls);
        turnToIntake(); // First turn to a position that allows robot to take in ball without being blocked.
        numIntakeBalls++;

        this.sorterList.add(new Artifact(color, sorter.getPosition()));
    }

    public void turnToIntake() { // turn sorter before intaking a ball
        telemetry.addData("current position", sorter.getPosition());

        if (sorter.getPosition() != 1) { // ensure the sorter cannot turn more than max
            double pos = this.sorterList.size() * 0.45;
            telemetry.addData("how many balls?", this.sorterList.size());
            sorter.setPosition(pos);
            telemetry.addData("turning to position", pos);
        }

        telemetry.update();
    }

    // quickFire fires a random ball
    public void quickFire() {
        if (this.sorterList.isEmpty()){
            telemetry.addLine("Nothing in sorter");
            telemetry.update();
            return;
        }

        sorter.setPosition(this.sorterList.get(0).getPosition());
        // push(); // TODO pusher
        telemetry.addData("Pushing out this ball", this.sorterList.get(0));
        telemetry.update();

        this.sorterList.remove(0);
    }

    // Test outtakeBall after quickFire
    // outtakeBall fires a ball from pattern
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

        sorter.setPosition(this.sorterList.get(ballIndexToRemoveFromSorter).getPosition());
        // push(); // TODO
        telemetry.addData("pushing this ball", colorToRemove);

        this.sorterList.remove(ballIndexToRemoveFromSorter);
        this.pattern.remove(0);
    }
}