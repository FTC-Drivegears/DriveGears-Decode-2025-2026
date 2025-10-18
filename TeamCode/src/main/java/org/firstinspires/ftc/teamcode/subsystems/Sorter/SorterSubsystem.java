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

    private final ColorSensor colourSensor;
    private final Telemetry telemetry;
    private final LinearOpMode opMode;

    private ArrayList<String> pattern;

    private ArrayList<Artifact> sorterList = new ArrayList<Artifact>();

    private int red;

    private int blue;

    private int green;

    private int alpha;

    private long lastColourDetectionTime;

    boolean detectedColour = false;

    public SorterSubsystem(Hardware hw, LinearOpMode opMode, Telemetry telemetry){
        this.sorter = hw.sorter;
        this.colourSensor = hw.colour;
        this.opMode = opMode;
        this.telemetry = telemetry;
        pattern = new ArrayList<>();
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

        if (sorter.getPosition() != 1) {
            // Ensures the sorter doesn't break
            sorter.setPosition(sorterList.size() * 0.45 );
        }
    }

    public void turnToColour(String color, Servo sorter) {
        double pos = 0;
        if (color.equals("Green")) {
            // Gets position of the first green it finds
            pos = sorterList.get(sorterList.indexOf("Green")).getPosition();
        }

        if (color.equals("Purple")) {
            // Gets position of the first purple it finds
            pos = sorterList.get(sorterList.indexOf("Purple")).getPosition();
        }

        // Turn sorter
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