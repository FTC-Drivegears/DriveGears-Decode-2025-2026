package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Artifact;
import java.util.ArrayList;

@TeleOp(name = "colourSensor")

public class DecodeAutoOpMode extends LinearOpMode {

    // Temporary order
    int red;
    int green;
    int blue;
    int alpha;
    boolean full = false;

    int numOfPattern = 1;
    //shows which pattern are we at
    ArrayList<String> pattern = new ArrayList<>();

    ArrayList<Artifact> sorter = new ArrayList<Artifact>();

    Servo servo = hardwareMap.get(Servo.class, "servo");

    @Override
    public void runOpMode() {
        ColorSensor colourSensor;
        colourSensor = hardwareMap.get(ColorSensor.class, "colour");
        colourSensor.enableLed(true);

        servo.setPosition(0);

        pattern.add("Purple");
        pattern.add("Green");
        pattern.add("Purple");

        waitForStart();

        while (opModeIsActive()) {
            red = colourSensor.red();
            green = colourSensor.green();
            blue = colourSensor.blue();
            alpha = colourSensor.alpha();

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.update();

            detectColour();

            turnSorter();

//            //**
//            if (String.valueOf(sorter.get(0)).equals(pattern.get(0))) {
//                // launch
//                servo.setPosition(servo.getPosition() - 0.5);
//            }
//            if (String.valueOf(sorter.get(1)).equals(pattern.get(1))) {
//                // launch
//                servo.setPosition(servo.getPosition() - 0.5);
//            }
//            *// launch



            // Finds the green ball in the list and gets the position of it (1,2,3)
            //double greenPosition = sorter.get(sorter.indexOf("Green")).getPosition();
        }
    }

    public void detectColour() {

        // Purple ball is detected
        if (blue > green) {
            sorter.add(new Artifact("Purple",sorter.size() + 1));
        }

        // Green ball is detected
        if (green - blue > 30) {
            sorter.add(new Artifact("Green", sorter.size() + 1));
        }

        //if sorter is full
        if (sorter.size() == 3) {
            full = true;
        }
    }

    public void turnSorter() {
        //If the sorter is full it stops
        if (full) {
            return;
        }

        if (servo.getPosition() != 1) {
            //make sure the servo don't break
            servo.setPosition(servo.getPosition() + 0.5);
        }
    }

    public void turnToColor(String color) {
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
    public void quickFire() {
        servo.setPosition(sorter.get(0).getPosition());
        sleep(500);
        //launch
        sorter.remove(0);

        servo.setPosition(sorter.get(0).getPosition());
        sleep(500);
        //launch
        sorter.remove(0);

        servo.setPosition(sorter.get(0).getPosition());
        sleep(500);
        //launch
        sorter.remove(0);
    }
}

/*
detect colour : 0
detect colour : 0.5
detect colour : 1
 */