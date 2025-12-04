package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.ArrayList;

public class SorterSubsystem {
    private final Servo sorter;

    private final ColorSensor colourSensor1;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    private ArrayList<String> pattern;

    private ArrayList<Artifact> sorterList = new ArrayList<Artifact>();

    private int red;

    private int blue;

    private int green;

    private int alpha;

    boolean detectedColor = false;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry){
        this.sorter = hw.sorter;
        this.colourSensor1 = hw.colour;
        this.opMode = opMode;
        this.telemetry = telemetry;
        pattern = new ArrayList<>();
    }

    public void detectColour() {
        /*int red = colourSensor1.red();
        int green = colourSensor1.green();
        int blue = colourSensor1.blue();
        int alpha = colourSensor1.alpha();*/
        //If the sorter is full it stops
        if (sorterList.size() == 3) {
            return;
        }

        // Purple ball is detected
        if (red / green > 0.6) {
            telemetry.addLine("Purple Detected");
            telemetry.update();
            if (!detectedColor) {
                detectedColor = true;
                sorterList.add(new Artifact("Purple", sorter.getPosition()));
                //turnsorter();
            } //green detected
        } else if (red / green < 0.55) {
            telemetry.addLine("Green Detected");
            telemetry.update();
            if (!detectedColor) {
                detectedColor = true;
                sorterList.add(new Artifact("Green", sorter.getPosition()));
                //turnsorter();
            }

        } else {
            opMode.sleep(300);
            detectedColor = false;
        }
    }

    public void turnsorter() {
        //If the sorterList is full it stops
        if (sorterList.size() == 3) {
            return;
        }

        if (sorter.getPosition() != 1) {
            //make sure the sorter doesn't break
            sorter.setPosition(sorterList.size() * 0.45 );
        }
    }

    public void turnToColour(String color, Servo sorter) {
        double pos = 0;
        if (color.equals("Green")) {
            //gets position of the first green it finds
            pos = sorterList.get(sorterList.indexOf("Green")).getPosition();
        }

        if (color.equals("Purple")) {
            //gets position of the first purple it finds
            pos = sorterList.get(sorterList.indexOf("Purple")).getPosition();
        }

        //move sorter
        sorter.setPosition(pos);
    }

    public void quickFire(Servo sorter) {
        sorter.setPosition(sorterList.get(0).getPosition());
        opMode.sleep(500);
        //launch
        sorterList.remove(0);

        sorter.setPosition(sorterList.get(0).getPosition());
        opMode.sleep(500);
        //launch
        sorterList.remove(0);

        sorter.setPosition(sorterList.get(0).getPosition());
        opMode.sleep(500);
        //launch
        sorterList.remove(0);
    }

    public int getNumBalls() {
        return sorterList.size();
    }

    public void setPattern(String art1, String art2, String art3){
        pattern.add(art1);
        pattern.add(art2);
        pattern.add(art3);
    }

    public int getRed() { return colourSensor1.red(); }

    public int getGreen(){
        return colourSensor1.green();
    }

    public int getBlue(){
        return colourSensor1.blue();
    }

    public int getAlpha(){
        return colourSensor1.alpha();
    }

    public double getPosition() {
        return sorter.getPosition();
    }

    public String getSorterIndex(int index) {
        return sorterList.get(index).getColour();
    }
}