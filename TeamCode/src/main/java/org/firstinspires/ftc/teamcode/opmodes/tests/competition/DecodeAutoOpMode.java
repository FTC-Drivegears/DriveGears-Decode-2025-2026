package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "colourSensor")

public class DecodeAutoOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorSensor colourSensor;
        colourSensor = hardwareMap.get(ColorSensor.class, "colour");
        Servo servo = hardwareMap.get(Servo.class, "servo");
        colourSensor.enableLed(true);

        servo.setPosition(0);

       waitForStart();

        while (opModeIsActive()) {
            int red = colourSensor.red();
            int green = colourSensor.green();
            int blue = colourSensor.blue();
            int alpha = colourSensor.alpha();

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.update();

            // Purple ball
            if(blue > green) {
                // Purple ball is detected
                servo.setPosition(1);
            }
            // Green ball
            else if(green - blue > 30) {
                // Green ball is detected
                servo.setPosition(0);
            }
            else {
                // No ball
                servo.setPosition(0.5);
            }
        }
    }
}