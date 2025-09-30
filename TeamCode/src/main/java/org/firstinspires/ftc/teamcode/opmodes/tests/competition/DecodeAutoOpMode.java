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

            // Finds the green ball in the list and gets the position of it (1,2,3)
            int greenPosition = sorter.get(sorter.indexOf("Green")).getPosition();
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
    }

    public void turnSorter() {
        /*
        Sorter turns
        If the sorter is full it stops
        */
        servo.setPosition(servo.getPosition() + 0.33);

        if (servo.getPosition() == 0.99) {
            // Stop getting balls
        }
    }
}

/*
detect colour (1)
turn (0.33)
detect colour (2)
turn (0.66)
detect colour (3)
turn (0.99)
FULL
 */