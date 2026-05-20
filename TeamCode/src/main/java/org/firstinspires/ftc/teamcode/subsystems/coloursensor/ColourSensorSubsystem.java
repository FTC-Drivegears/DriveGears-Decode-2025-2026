package org.firstinspires.ftc.teamcode.subsystems.coloursensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;

public class ColourSensorSubsystem {
    private boolean lastArtifactPresent = false;
    private Hardware hw;
    private SorterSubsystem sorterSubsystem;
    private ColorSensor colourSensor1;
    private ColorSensor colourSensor2;
    private Servo sorter;
    private Servo leftLight;

    private int colour_threshold = 80;
    private Artifact[] sorterList;
    private int red, green, blue, alpha;
    private int red2, green2, blue2, alpha2;

    public ColourSensorSubsystem(HardwareMap hardwareMap, Hardware hw, SorterSubsystem sorterSubsystem) {
        this.hw = hw;
        this.sorterSubsystem = sorterSubsystem;
        this.colourSensor1 = hardwareMap.get(ColorSensor.class, "colour1");
        this.colourSensor2 = hardwareMap.get(ColorSensor.class, "colour2");

        this.sorter = hw.sorter;
        this.leftLight = hw.leftLight;

        this.sorterList = sorterSubsystem.getSorterList();

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

        if (sorterSubsystem.getArtifactCount() == 3) return;

        boolean artifactPresent = alpha > 150 || alpha2 > 150;
        boolean artifactCleared = alpha < colour_threshold && alpha2 < colour_threshold;

        boolean purple_colour1 = percentMoreThan(blue, 20, red) && percentMoreThan(blue, 20, green);
        boolean purple_colour2 = percentMoreThan(blue2, 20, red2) && percentMoreThan(blue2, 20, green2);

        boolean green_colour1 = percentMoreThan(green, 20, red) && percentMoreThan(green, 20, blue);
        boolean green_colour2 = percentMoreThan(green2, 20, red2) && percentMoreThan(green2, 20, blue2);

        if (intakeOn && artifactPresent && !lastArtifactPresent) {
            if (purple_colour1 || purple_colour2) {
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Purple");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.7);
                if (sorterSubsystem.getSorterPos() < 3)
                    sorterSubsystem.manualSpin();
                lastArtifactPresent = true;
            } else if (green_colour1 || green_colour2) {
                sorterList[sorterSubsystem.getSorterPos()] = new Artifact("Green");
                sorterSubsystem.setArtifactCount(sorterSubsystem.getArtifactCount() + 1);
                leftLight.setPosition(0.5);
                if (sorterSubsystem.getSorterPos() < 3)
                    sorterSubsystem.manualSpin();
                lastArtifactPresent = true;
            }
        }
        if (artifactCleared) {
            lastArtifactPresent = false;
        } else {
            lastArtifactPresent = artifactPresent;
        }
    }

        private boolean percentMoreThan (double colour1, double percent, double colour2) {
            return (colour1 / colour2 ) >= (percent/100 + 1);
        }
        public int getRed () { return red; }
        public int getGreen () {return green; }
        public int getBlue () { return blue; }
        public int getAlpha () { return alpha; }
        public int getRed2 () { return red2; }
        public int getGreen2 () {return green2; }
        public int getBlue2 () { return blue2; }
        public int getAlpha2 () { return alpha2; }

}




