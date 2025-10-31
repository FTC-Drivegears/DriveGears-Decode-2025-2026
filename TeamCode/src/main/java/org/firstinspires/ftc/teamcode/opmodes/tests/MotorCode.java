package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem;

@TeleOp(name = "MotorCode", group = "TeleOp")
public class MotorCode extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private ElapsedTime resetTimer;
    private Hardware hw;
    private double theta;

    @Override
    public void runOpMode() throws InterruptedException {
        int buttonCounter = 0;
        boolean buttonCondition;
        hw = Hardware.getInstance(hardwareMap);
        CRServo servo = hardwareMap.get(CRServo.class, "servo");
        mecanumSubsystem = new MecanumSubsystem(hw);
        resetTimer = new ElapsedTime();

        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {
            theta = mecanumSubsystem.fieldOrientedMove(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (gamepad1.a) {
                buttonCounter++;
                buttonCondition = true;
                while (buttonCondition) {
                    servo.setPower(-1);
                    if (buttonCounter % 2 == 0) {
                        buttonCondition = false;
                    }
                }
            }
            if (gamepad1.b) {//reject if stuck
                buttonCounter++;
                buttonCondition = true;
                while (buttonCondition) {
                    servo.setPower(-1);
                    if (buttonCounter % 2 == 0) {
                        buttonCondition = false;
                    }
                }
            }
            processTelemetry();

            if (gamepad1.start) {
                mecanumSubsystem.resetPinPointOdometry();
            }
        }
    }

    public void processTelemetry() {        //add telemetry messages here
        telemetry.addData("resetTimer: ", resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("theta", theta);
        telemetry.update();
    }
}