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

    public Telemetry telemetry;

    private final ElapsedTime detectColorTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        this.colourSensor = hw.colourSensor;
        this.colourSensor2 = hw.colourSensor2;

        hw = Hardware.getInstance(hardwareMap);

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

            int alpha = colourSensor.alpha();
            int alpha2 = colourSensor2.alpha();

            detectColorTime.reset();
            while (!hasDetectedColor && detectColorTime.milliseconds() <= 2000) {
                if (isPurple(red, green, blue, alpha) || isPurple(red2, green2, blue2, alpha2)) {
                    telemetry.addLine("Purple Detected");
                    hasDetectedColor = true;
                    detectedPurple = true;
                } else if (isGreen(red, green, blue, alpha) || isGreen(red2, green2, blue2, alpha2)) {
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

    private boolean isPurple(int red, int green, int blue, int alpha) {
        return red > 50 && red < 65 && green < 95 && green > 80 && blue < 95 && blue > 78 && alpha < 85 && alpha > 70;
    }

    private boolean isGreen(int red, int green, int blue, int alpha){
        return red < 55 && red > 40 && green < 110 && green > 90 && blue < 90 && blue > 70 && alpha < 85 && alpha > 65;
    }
}
