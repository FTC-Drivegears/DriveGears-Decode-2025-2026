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

        boolean detectedPurple = false;
        boolean detectedGreen = false;

        while (opModeIsActive()) {
            detectColorTime.reset();
            boolean hasDetectedColor = false;
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
                    if (isPurple(red, green, blue, alpha) || isPurple(red2, green, blue2, alpha2)) {
                        telemetry.addLine("Purple Detected");
                        hasDetectedColor = true;
                        detectedPurple = true;
                    } else if (isGreen(red,green,blue,alpha) || isGreen(red2, green2, blue2, alpha)) {
                        telemetry.addLine("Green Detected");
                        hasDetectedColor = true;
                        detectedGreen = true;

                    }

                }
                telemetry.addData("Did it detect color?", hasDetectedColor);
                telemetry.addData("Am I seeing purple?", detectedPurple);
                telemetry.addData("Am I seeing green?", detectedGreen);
                telemetry.addData("red", red);
                telemetry.addData("red2", red2);
                telemetry.addData("green", green);
                telemetry.addData("green2", green2);
                telemetry.addData("blue2", blue2);
                telemetry.addData("blue", blue);
                telemetry.update();
            }
        }
    }

    private boolean isGreen(int red, int green, int blue, int alpha) {
        //return green > red && green > blue && green >= 255;
        return red > 50 && red < 65 && green < 95 && green > 80 && blue < 95 && blue > 78 && alpha < 85 && alpha > 70;
    }


    boolean isPurple ( int red, int green, int blue, int alpha){
        //return blue > green && alpha < 700;
        return red < 55 && red > 40 && green < 110 && green > 90 && blue < 90 && blue > 70 && alpha < 85 && alpha > 65;
    }
}