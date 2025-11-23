package org.firstinspires.ftc.teamcode.subsystems.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Artifact;
import java.util.ArrayList;

public class SorterSubsystem {
    public final Servo sorter;

    private final ColorSensor colourSensor;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    private ArrayList<Artifact> sorterList = new ArrayList<Artifact>();

    private int red;
    private int blue;
    private int green;
    private int alpha;

    private long lastColourDetectionTime;
    private long lastColourTurnTime;

    boolean detectedColour = false;

    private static double pos1 = 0.085;
    private static double pos2 = 0.515;
    private static double pos3 = 0.96;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry){
        this.sorter = hw.sorter;
        this.colourSensor = hw.colour;
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public void detectColour() {
        red = colourSensor.red();
        green = colourSensor.green();
        blue = colourSensor.blue();
        alpha = colourSensor.alpha();

        // If the sorter is full it stops
        if (sorterList.size() == 3) {
            return;
        }

        // Purple ball is detected
        if (red > 50 && red < 65 && green < 95 && green > 80 && blue < 95 && blue > 78 && alpha < 85 && alpha > 70) {
            telemetry.addLine("Purple Detected");
            telemetry.update();
            if (!detectedColour) {
                detectedColour = true;
                sorterList.add(new Artifact("Purple", sorter.getPosition()));
                turnSorter();
            }
        }

        // Green ball is detected
        else if (red < 55 && red > 40 && green < 110 && green > 90 && blue < 90 && blue > 70 && alpha < 85 && alpha > 65) {
            telemetry.addLine("Green Detected");
            telemetry.update();
            if (!detectedColour) {
                detectedColour = true;
                sorterList.add(new Artifact("Green", sorter.getPosition()));
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

    public void turnSorter() {
        // If the sorterList is full it stops
        if (sorterList.size() == 3) {
            return;
        }
        if (sorter.getPosition() == pos2) {
            sorter.setPosition(pos3);
        }
        if (sorter.getPosition() == pos1) {
            sorter.setPosition(pos2);
        }
    }

    public void turnToColour(String colour, Servo sorter) {
        double pos = 0;
        if (colour.equals("Green")) {
            // Gets position of the first green it finds
            pos = sorterList.get(sorterList.indexOf("Green")).getPosition();
        }

        if (colour.equals("Purple")) {
            // Gets position of the first purple it finds
            pos = sorterList.get(sorterList.indexOf("Purple")).getPosition();
        }

        // Turn sorter
        sorter.setPosition(pos);
    }

    public void quickFire(Servo sorter) {
        sorter.setPosition(sorterList.get(0).getPosition());
        opMode.sleep(500);
        // launch
        sorterList.remove(0);

        sorter.setPosition(sorterList.get(0).getPosition());
        opMode.sleep(500);
        // launch
        sorterList.remove(0);

        sorter.setPosition(sorterList.get(0).getPosition());
        opMode.sleep(500);
        // launch
        sorterList.remove(0);
    }

    public int getNumBalls() {
        return sorterList.size();
    }

    public int getRed(){
        return red;
    }

    public int getGreen(){
        return green;
    }

    public int getBlue(){
        return blue;
    }

    public int getAlpha(){
        return alpha;
    }

    public double getPosition() {
        return sorter.getPosition();
    }

    public String getSorterIndex(int index) {
        return sorterList.get(index).getColour();
    }
}