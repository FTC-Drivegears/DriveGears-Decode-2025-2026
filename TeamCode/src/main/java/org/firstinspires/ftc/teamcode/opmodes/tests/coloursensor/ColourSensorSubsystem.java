package org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.ArrayList;

public class ColourSensorSubsystem {
    private boolean lastArtifactPresent = false;
    private Hardware hw;
    private ColorSensor colourSensor1;
    private ColorSensor colourSensor2;
    private Servo sorter;
    private Servo light;

    private boolean detectedColor = false;
    private ArrayList<Artifact> sorterList = new ArrayList<>();

    private int red, green, blue, alpha;
    private int red2, green2, blue2, alpha2;

    public ColourSensorSubsystem(HardwareMap hardwareMap, Hardware hw) {
        this.hw = hw;
        this.colourSensor1 = hardwareMap.get(ColorSensor.class, "colour1");
        this.colourSensor2 = hardwareMap.get(ColorSensor.class, "colour2");

        this.sorter = hw.sorter;
        this.light = hw.light;

        colourSensor1.enableLed(true);
        colourSensor2.enableLed(true);
    }

    public void update(boolean intakeOn) {
        red = colourSensor1.red();
        green = colourSensor1.green();
        blue = colourSensor1.blue();
        alpha = colourSensor1.alpha();

        red2 = colourSensor2.red();
        green2 = colourSensor2.green();
        blue2 = colourSensor2.blue();
        alpha2 = colourSensor2.alpha();

        if (!intakeOn) return;
        if (sorterList.size() == 3) return;

        boolean artifactPresent = alpha > 150 || alpha2 > 150;
        if (artifactPresent && !lastArtifactPresent) {

            if (sorterList.size() < 3) {

                // PURPLE
                if (blue > red + 30 && blue > green + 30) {
                    sorterList.add(new Artifact("Purple", sorter.getPosition()));
                    light.setPosition(0.7);
                    turnSorter();
                }

                // GREEN
                else if (green > blue + 30 && green > red + 30) {
                    sorterList.add(new Artifact("Green", sorter.getPosition()));
                    light.setPosition(0.5);
                    turnSorter();
                }

                else {
                    light.setPosition(0);
                }
            }
        }
        lastArtifactPresent = artifactPresent;
    }

    private void turnSorter() {
        double pos = Math.min(sorterList.size() * 0.43, 1.0);
        sorter.setPosition(pos);
    }

    public int getRed() { return red; }

    public int getGreen() { return green; }

    public int getBlue() { return blue; }
    public int getAlpha() { return alpha; }
    public int getRed2() { return red2; }
    public int getGreen2() { return green2; }
    public int getBlue2() { return blue2; }
    public int getAlpha2() { return alpha2; }
    public int getCount() { return sorterList.size(); }
}

