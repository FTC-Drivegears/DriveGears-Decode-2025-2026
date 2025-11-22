package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "HalloweenRobot", group = "TeleOp")
public class HalloweenRobot extends LinearOpMode {
    private DcMotor shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotor.class,"shooter");

        boolean previousAState = false;
        boolean currentAState;
        boolean isMotorOn = false;

        while (opModeInInit()){
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            currentAState = gamepad1.a;
            if (currentAState && !previousAState) {
                isMotorOn = !isMotorOn;

                if (isMotorOn) {
                    shooter.setPower(-1);
                } else {
                    shooter.setPower(0);
                }
            }
            previousAState = currentAState;
        }
    }
}
