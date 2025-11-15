package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "turret")
public class TurretTest extends LinearOpMode {
    CRServo servo;
    private DcMotor servoEncoder;
    double voltage;

    public void runOpMode() throws InterruptedException {
        servoEncoder = hardwareMap.get(DcMotor.class,"servo");
        servo = hardwareMap.get(CRServo.class, "crservo");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPower(1);
            voltage = servoEncoder.getCurrentPosition();

            telemetry.addData("voltage", voltage);
            telemetry.update();
        }
    }
}
