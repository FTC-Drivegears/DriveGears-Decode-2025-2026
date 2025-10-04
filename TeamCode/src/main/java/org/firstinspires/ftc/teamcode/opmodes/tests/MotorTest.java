package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx lb = hardwareMap.get(DcMotorEx.class, "lb");
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "rb");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                lf.setPower(1);
            } else {
                lf.setPower(0);
            }

            if (gamepad1.b) {
                rf.setPower(1);
            } else {
                rf.setPower(0);
            }

            if (gamepad1.x) {
                lb.setPower(1);
            } else {
                lb.setPower(0);
            }

            if (gamepad1.y) {
                rb.setPower(1);
            } else {
                rb.setPower(0);
            }
//            if (gamepad1.a) {
//                lf.setPower(gamepad1.a ? 1.0 : 0.0);  // A -> LF
//                rf.setPower(gamepad1.b ? 1.0 : 0.0);  // B -> RF
//                lb.setPower(gamepad1.x ? 1.0 : 0.0);  // X -> LB
//                rb.setPower(gamepad1.y ? 1.0 : 0.0);  // Y -> RB
//            }
            telemetry.addData("LF Power", lf.getPower());
            telemetry.addData("RF Power", rf.getPower());
            telemetry.addData("LB Power", lb.getPower());
            telemetry.addData("RB Power", rb.getPower());
            telemetry.update();
        }
    }
}