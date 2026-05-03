package org.firstinspires.ftc.teamcode.subsystems.coloursensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.ArrayList;

public class ColourSensorSubsystem {
    private boolean lastArtifactPresent = false;
    private Hardware hw;
    private SorterSubsystem sorterSubsystem;
    private ColorSensor colourSensor1;
    private ColorSensor colourSensor2;
    private Servo sorter;
    private Servo light;

    private int colour_threshold = 80;
    private Artifact[] sorterList;
    private int[] artifactCount;
    private int red, green, blue, alpha;
    private int red2, green2, blue2, alpha2;

    public ColourSensorSubsystem(HardwareMap hardwareMap, Hardware hw, SorterSubsystem sorterSubsystem) {
        this.hw = hw;
        this.colourSensor1 = hardwareMap.get(ColorSensor.class, "colour1");
        this.colourSensor2 = hardwareMap.get(ColorSensor.class, "colour2");

        this.sorter = hw.sorter;
        this.light = hw.light;

        this.sorterList = sorterSubsystem.getSorterList();
        this.artifactCount = sorterSubsystem.getArtifactCount();

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

        if (artifactCount[0] == 3) return;

        boolean artifactPresent = alpha > 150 || alpha2 > 150;
        boolean artifactCleared = alpha < colour_threshold && alpha2 < colour_threshold;

        boolean purple_colour1 = blue > red + 50 && blue > green + 30;
        boolean purple_colour2 = blue2 > red2 + 50 && blue2 > green2 + 30;

        boolean green_colour1 = green > blue + 70 && green > red + 70;
        boolean green_colour2 = green2 > blue2 + 70 && green2 > red2 + 70;

        if (intakeOn && artifactPresent && !lastArtifactPresent) {
            if (purple_colour1 || purple_colour2) {
                sorterList[artifactCount[0]] = new Artifact("Purple");
                artifactCount[0]++;
                light.setPosition(0.7);
                if (artifactCount[0] < 3)
                    turnSorter();
                lastArtifactPresent = true;
            } else if (green_colour1 || green_colour2) {
                sorterList[artifactCount[0]] = new Artifact("Green");
                artifactCount[0]++;
                light.setPosition(0.5);
                if (artifactCount[0] < 3)
                    turnSorter();
                lastArtifactPresent = true;
            }
        }
        if (artifactCleared) {
            lastArtifactPresent = false;
        } else {
            lastArtifactPresent = artifactPresent;
        }
    }

        public int getRed () { return red; }
        public int getGreen () {return green; }
        public int getBlue () { return blue; }
        public int getAlpha () { return alpha; }
        public int getRed2 () { return red2; }
        public int getGreen2 () {return green2; }
        public int getBlue2 () { return blue2; }
        public int getAlpha2 () { return alpha2; }
        public int getCount () { return artifactCount[0]; }

        private void turnSorter () {
            double pos = Math.min(artifactCount[0] * 0.43, 1.0);
            sorter.setPosition(pos);
    }
}




