
//this is for teleop
package org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
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
        //this.colourSensor1 = hw.colourSensor1;
        //this.colourSensor2 = hw.colourSensor2;

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
            float hue = JavaUtil.colorToHue(colors.toColor());
            float hue2 = JavaUtil.colorToHue(colors2.toColor());

            if (hue < 163 && hue > 150 || hue2 < 163 && hue2 > 150){
                telemetry.addData("Color", "Green");
            }
            else if (hue < 350 && hue > 165 || hue2 < 350 && hue2 > 165){
                telemetry.addData("Color", "Purple");
            }
            else{
                telemetry.addLine("Nothing is here");
            }


            telemetry.addData("Red:", colors.red);
            telemetry.addData("Red2:", colors2.red);
            telemetry.addData("Green:", colors.green);
            telemetry.addData("Green2:", colors2.green);
            telemetry.addData("Blue:", colors.blue);
            telemetry.addData("Blue2:", colors2.blue);

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

