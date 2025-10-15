package org.firstinspires.ftc.teamcode.opmodes.tests;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private DcMotorEx motorIntake;

    enum AUTO_STATE {
        SCAN_OBELISK,
        BALL_PICKUP1,
        BALL_PICKUP2,
        FINISH
    }

    enum AUTO_PATTERN {
        GPP,
        PGP,
        PPG,

    }


    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        motorIntake = hardwareMap.get(DcMotorEx.class, "externalIntake");
        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .build();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        AUTO_STATE autoState = AUTO_STATE.SCAN_OBELISK;
        waitForStart();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            switch (autoState) {
                case SCAN_OBELISK:
                    if (tagProcessor.getDetections().size() > 0) {
                        AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    }

                case BALL_PICKUP1:
                    if (mecanumCommand.moveToPos(30, 0, 0)) {
                        motorIntake.setPower(1);
                        sleep(800);
                        autoState = AUTO_STATE.BALL_PICKUP2;
                    }
                    break;

                case BALL_PICKUP2:
                    if (mecanumCommand.moveToPos(30, 10, 0)) {
                        motorIntake.setPower(1);
                        sleep(800);
                        autoState = AUTO_STATE.FINISH;
                    }
                    break;

                case FINISH:
                    motorIntake.setPower(0);  // Ensure intake is stopped
                    stopRobot();
                    break;
            }
        }

        private void stopRobot () {
            mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
        }
    }
}