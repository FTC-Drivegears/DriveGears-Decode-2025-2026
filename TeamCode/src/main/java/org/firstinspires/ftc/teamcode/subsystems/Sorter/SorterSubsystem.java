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
    private int countRemovedBalls;

    private ArrayList<Character> sorterList;
    private int red;
    private int blue;
    private int green;
    private int alpha;

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

    public void turnToOuttake() {
        if (this.pattern.isEmpty()){
            telemetry.addLine("Pattern is empty");
            telemetry.update();
            return;
        }

        char colorToRemove = this.pattern.get(0);

        int ballIndexToRemoveFromSorter = -1;

        for (int i = 0; i <= this.sorterList.size(); i++){

            if (this.sorterList.get(i) == colorToRemove){

                ballIndexToRemoveFromSorter = i;
                break;
            }
        }

        sorter.setPosition(PUSHER_POSITION);

        // push(); // TODO pusher

        if (ballIndexToRemoveFromSorter == -1){
            telemetry.addLine("No balls to outtake");
            telemetry.update();
            return;
        }
        this.sorterList.remove(ballIndexToRemoveFromSorter);
        this.pattern.remove(0);
    }

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry, String pattern) {
        this.sorter = hw.sorter;
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.pattern = new ArrayList<>();
        sorterList = new ArrayList<>();

    }

    public void intakeBall(char color){
        // fail-safe
        if (sorterList.size() == MAX_NUM_BALLS){
            telemetry.addLine("Cannot intake any more balls, max capacity");
            telemetry.update();
            return;
        }

        sorterList.add(color);
        turnToIntake();
    }

    public void turnToIntake() {
        if (sorter.getPosition() != 1) {
            //make sure the sorter doesn't break
            sorter.setPosition(sorterList.size() * 0.45 );
        }
    }

////    public void quickFire(Servo sorter) {
//////        sorter.setPosition(sorterList.get(0).getPosition());
////        opMode.sleep(500);
////        //launch
////        sorterList.remove(0);
////
//////        sorter.setPosition(sorterList.get(0).getPosition());
////        opMode.sleep(500);
////        //launch
////        sorterList.remove(0);
////
//////        sorter.setPosition(sorterList.get(0).getPosition());
////        opMode.sleep(500);
////        //launch
////        sorterList.remove(0);
////    }
//
//    public int getNumBalls() {
//        return sorterList.size();
//    }
}