//this is for teleop
package org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "ColorSensorTest")
public class ColorSensorTest extends LinearOpMode {
    private Hardware hw;
    public ColorSensor colourSensor;
    public ColorSensor colourSensor2;

    private Telemetry telemetry;

    private final ElapsedTime detectColorTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        this.colourSensor = hw.colourSensor;
        this.colourSensor2 = hw.colourSensor2;

        while (opModeInInit()) {
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();

        detectColorTime.reset();

        boolean hasDetectedColor = false;
        boolean detectedPurple = false;
        boolean detectedGreen = false;

        while (!hasDetectedColor && detectColorTime.milliseconds() <= 2000) {
            int red = colourSensor.red();
            int red2 = colourSensor2.red();

            int green = colourSensor.green();
            int green2 = colourSensor2.green();

            int blue = colourSensor.blue();
            int blue2 = colourSensor2.blue();

            detectColorTime.reset();
            while (!hasDetectedColor && detectColorTime.milliseconds() <= 2000) {
                if (isPurple(red, green, blue) || isPurple(red2, green2, blue2)) {
                    telemetry.addLine("Purple Detected");
                    hasDetectedColor = true;
                    detectedPurple = true;
                } else if (isGreen(red, green, blue) || isGreen(red2, green2, blue2)) {
                    telemetry.addLine("Green Detected");
                    hasDetectedColor = true;
                    detectedGreen = true;
                }
            }
            telemetry.addData("Did it detect color?", hasDetectedColor);
            telemetry.addData("Am I seeing purple?", detectedPurple);
            telemetry.addData("Am I seeing green?", detectedGreen);
            telemetry.update();
        }
    }

    boolean isPurple(int red, int green, int blue) {
        return red > 99 && red  < 181 && green > 79 && green < 96 && blue > 77 && blue < 96;
    }

    boolean isGreen(int red, int green, int blue) {
        return red > 19 && red < 81 && green > 99 && green < 256 && blue > 19 && blue < 81;
    }
}
