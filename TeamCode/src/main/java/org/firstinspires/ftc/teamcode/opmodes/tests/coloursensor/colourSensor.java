package org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Artifact;
import java.util.ArrayList;

@TeleOp(name = "colourSensor")

public class colourSensor extends LinearOpMode {
    private Hardware hw;
    private Servo light;
    int red;
    int green;
    int blue;
    int alpha;

    int red2;
    int green2;
    int blue2;
    int alpha2;
    boolean detectedColor = false;
    ArrayList<String> pattern = new ArrayList<>();
    ArrayList<Artifact> sorter = new ArrayList<Artifact>();

    @Override
    public void runOpMode() {
        hw = Hardware.getInstance(hardwareMap);
        ColorSensor colourSensor1;
        ColorSensor colourSensor2;
        colourSensor1 = hardwareMap.get(ColorSensor.class, "colour1");
        colourSensor2 = hardwareMap.get(ColorSensor.class, "colour2");
        Servo servo = hardwareMap.get(Servo.class, "sorter");
        colourSensor1.enableLed(true);
        colourSensor2.enableLed(true);

        servo.setPosition(0);

        pattern.add("Purple");
        pattern.add("Green");
        pattern.add("Purple");
        light = hw.light;

        waitForStart();

        while (opModeIsActive()) {

            red = colourSensor1.red();
            green = colourSensor1.green();
            blue = colourSensor1.blue();
            alpha = colourSensor1.alpha();

            red2 = colourSensor2.red();
            green2 = colourSensor2.green();
            blue2 = colourSensor2.blue();
            alpha2 = colourSensor2.alpha();
            detectColour(servo);

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("Red2", red2);
            telemetry.addData("Green2", green2);
            telemetry.addData("Blue2", blue2);
            telemetry.addData("Alpha2", alpha2);

            telemetry.addData("Amount of Artifacts", sorter.size());
            telemetry.addData("position", servo.getPosition());
            telemetry.update();
        }
    }

    public void detectColour(Servo servo) {
        //If the sorter is full it stops
        if (sorter.size() == 3) {
            return;
        }

        // Purple ball is detected
//        if (blue > red + 30 && blue > green + 30 && alpha < 700) {
//            if (blue2 > red2 + 30 && blue2 > green2 + 30 && alpha2 < 700) {
        if (blue > green && alpha < 700 && blue2 > green2 && alpha2 < 700) {
            if (detectedColor == false) {
                detectedColor = true;
                sorter.add(new Artifact("Purple", servo.getPosition()));
                light.setPosition(0.7);
                telemetry.addLine("Purple Detected");
                turnSorter(servo);
            }
             //green detected
//]
//         if  (green > blue + 30 && green > red + 30 && alpha > 70 && alpha < 110) {
//       if (green2 > blue2 + 30 && green2 > red2 + 30 && alpha2 > 70 && alpha2 < 110) {
        } else if (green - blue > 20 && green2 - blue2 > 20) {
           if (detectedColor == false) {
               detectedColor = true;
               sorter.add(new Artifact("Green", servo.getPosition()));
               light.setPosition(0.5);
                telemetry.addLine("Green Detected");
               turnSorter(servo);
       }

        } else {
            detectedColor = false;
        }

        telemetry.update();
    }

    public void turnSorter(Servo servo) {
        //If the sorter is full it stops
            if (sorter.size() == 3) {
            return;
        }

        if (servo.getPosition() != 1) {
            //make sure the servo doesn't break
            servo.setPosition(sorter.size() * 0.45 );
        }
    }

    public void turnToColour(String color, Servo servo) {
        double pos = 0;
        if (color.equals("Green")) {
            //gets position of the first green it finds
            pos = sorter.get(sorter.indexOf("Green")).getPosition();
        }

        if (color.equals("Purple")) {
            //gets position of the first purple it finds
            pos = sorter.get(sorter.indexOf("Purple")).getPosition();
        }

        //move servo
        servo.setPosition(pos);
    }

}