package org.firstinspires.ftc.teamcode.subsystems.coloursensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;

public class ColourSensorSubsystem {

    private boolean lastArtifactPresent = false;
    private Hardware hw;
    private SorterSubsystem sorterSubsystem;
    private NormalizedColorSensor colourSensor1;
    private NormalizedColorSensor colourSensor2;
    private Servo sorter;
    private Servo leftLight;

    private float alpha_threshold = 0.15f;
    private Artifact[] sorterList;
    private float red, green, blue, alpha;
    private float red2, green2, blue2, alpha2;
    private float lastRed = 0, lastGreen = 0, lastBlue = 0, lastAlpha = 0;

    public ColourSensorSubsystem(HardwareMap hardwareMap, Hardware hw, SorterSubsystem sorterSubsystem) {
        this.hw = hw;
        this.sorterSubsystem = sorterSubsystem;
        this.colourSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colour1");
        this.colourSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colour2");
        this.sorter    = hw.sorter;
        this.leftLight = hw.leftLight;
        this.sorterList = sorterSubsystem.getSorterList();
    }

    public void update() {
        NormalizedRGBA colors1 = colourSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = colourSensor2.getNormalizedColors();

        red   = colors1.red;   green = colors1.green;
        blue  = colors1.blue;  alpha = colors1.alpha;
        red2  = colors2.red;   green2 = colors2.green;
        blue2 = colors2.blue;  alpha2 = colors2.alpha;

        if (sorterSubsystem.getArtifactCount() == 3) return;

        boolean artifactPresent = alpha > 0.4f || alpha2 > 0.4f;
        boolean artifactCleared = alpha < alpha_threshold && alpha2 < alpha_threshold;

        boolean purple_colour1 = percentMoreThan(blue,  10, red)  && percentMoreThan(blue,  10, green);
        boolean purple_colour2 = percentMoreThan(blue2, 10, red2) && percentMoreThan(blue2, 10, green2);
        boolean green_colour1  = percentMoreThan(green,  20, red)  && percentMoreThan(green,  20, blue);
        boolean green_colour2  = percentMoreThan(green2, 20, red2) && percentMoreThan(green2, 20, blue2);

        boolean curPosIsEmpty = sorterList[sorterSubsystem.getSorterPos()].getColour().equals("None");

        if (curPosIsEmpty && artifactPresent && !lastArtifactPresent) {
            if (purple_colour1 || purple_colour2) {
                lastRed = red; lastGreen = green; lastBlue = blue; lastAlpha = alpha;
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Purple");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.7);
                if (sorterSubsystem.getSorterPos() < 3) sorterSubsystem.manualSpin();
                lastArtifactPresent = true;
            } else if (green_colour1 || green_colour2) {
                lastRed = red; lastGreen = green; lastBlue = blue; lastAlpha = alpha;
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Green");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.5);
                if (sorterSubsystem.getSorterPos() < 3) sorterSubsystem.manualSpin();
                lastArtifactPresent = true;
            }
        }

        if (artifactCleared) {
            lastArtifactPresent = false;
        } else {
            lastArtifactPresent = artifactPresent;
        }
    }

    private boolean percentMoreThan(double colour1, double percent, double colour2) {
        return (colour1 / colour2) >= (percent / 100 + 1);
    }

    /**
     * Returns true if either sensor detects an object regardless of colour.
     * Call after update() each loop.
     */
    public boolean isBallPresent() { return alpha > 0.4f || alpha2 > 0.4f; }
    public float getRed()    { return red; }
    public float getGreen()  { return green; }
    public float getBlue()   { return blue; }
    public float getAlpha()  { return alpha; }
    public float getRed2()   { return red2; }
    public float getGreen2() { return green2; }
    public float getBlue2()  { return blue2; }
    public float getAlpha2() { return alpha2; }
    public float[] getLastValues() { return new float[]{ lastRed, lastGreen, lastBlue, lastAlpha }; }
}