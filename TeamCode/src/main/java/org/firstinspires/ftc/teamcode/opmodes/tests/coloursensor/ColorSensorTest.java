
//this is for teleop
package org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "ColorSensorTest")
public class ColorSensorTest extends LinearOpMode {
    private Hardware hw;
    public NormalizedColorSensor colourSensor1;
    public NormalizedColorSensor colourSensor2;
    private final ElapsedTime detectColorTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        hw = Hardware.getInstance(hardwareMap);
//        this.colourSensor1 = hw.colourSensor1;
//        this.colourSensor2 = hw.colourSensor2;

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

            NormalizedRGBA colors = colourSensor1.getNormalizedColors();
            NormalizedRGBA colors2 = colourSensor2.getNormalizedColors();

            boolean isPurple = (colors.blue > 0.25f) &&
                    (colors.red > 0.15f) &&
                    (colors.blue > colors.green * 1.2f);

            boolean isPurple2 = (colors2.blue > 0.25f)
                    && (colors2.red > 0.15f)
                    && (colors2.blue > colors2.green * 1.2f);

            boolean isGreen = (colors.green >0.2f)
                    && (colors.green > colors.red * 1.3f)
                    && (colors.green > colors.blue * 1.3f)
                    && (colors.red < 0.5f) && (colors.blue < 0.5f);

            boolean isGreen2 = (colors2.green >0.2f)
                    && (colors2.green > colors2.red * 1.3f)
                    && (colors2.green > colors2.blue * 1.3f)
                    && (colors2.red < 0.5f) && (colors2.blue < 0.5f);

            telemetry.addData("Red:", colors.red);
            telemetry.addData("Red2:", colors2.red);
            telemetry.addData("Green:", colors.green);
            telemetry.addData("Green2:", colors2.green);
            telemetry.addData("Blue:", colors.blue);
            telemetry.addData("Blue2:", colors2.blue);
            telemetry.addData("Am I seeing purple", isPurple);
            telemetry.addData("Am I seeing purple on second cs", isPurple2);
            telemetry.addData("Am I seeing purple", isGreen);
            telemetry.addData("Am I seeing purple on second cs", isGreen2);

            telemetry.update();

//            int red = colourSensor.red();
//            int red2 = colourSensor2.red();
//
//            int green = colourSensor.green();
//            int green2 = colourSensor2.green();
//
//            int blue = colourSensor.blue();
//            int blue2 = colourSensor2.blue();
//
//            int alpha = colourSensor.alpha();
//            int alpha2 = colourSensor2.alpha();
//
//            detectColorTime.reset();
//            if (isPurple(red, green, blue, alpha) || isPurple(red2, green, blue2, alpha2)) {
//                telemetry.addLine("Purple Detected");
//                hasDetectedColor = true;
//                detectedPurple = true;
//            } else if (isGreen(red,green,blue,alpha) || isGreen(red2, green2, blue2, alpha)) {
//                telemetry.addLine("Green Detected");
//                hasDetectedColor = true;
//                detectedGreen = true;
//
            }
        }
    }

//    private boolean isGreen(int red, int green, int blue, int alpha) {
//        //return green > red && green > blue && green >= 255;
//        return red > 50 && red < 65 && green < 95 && green > 80 && blue < 95 && blue > 78 && alpha < 85 && alpha > 70;
//    }
//
//
//    boolean isPurple ( int red, int green, int blue, int alpha){
//        //return blue > green && alpha < 700;
//        return red < 55 && red > 40 && green < 110 && green > 90 && blue < 90 && blue > 70 && alpha < 85 && alpha > 65;
//    }

