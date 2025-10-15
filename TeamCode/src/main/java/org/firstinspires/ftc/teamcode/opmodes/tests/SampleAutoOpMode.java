package org.firstinspires.ftc.teamcode.opmodes.tests;
import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
        BALL_PICKUP,

        //SORTER
        FINISH

    }

//    enum OBELISK_SORTER {
//        GPP,
//        PGP,
//        PPG
//
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        motorIntake = hardwareMap.get(DcMotorEx.class, "externalIntake");
        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
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
                        sleep(3000);
                    }

                    if (mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.BALL_PICKUP;
                    }
                    break;

                case BALL_PICKUP:
                    if (mecanumCommand.moveToPos(30, 0, 0)) {
                        autoState = AUTO_STATE.FINISH;
                        motorIntake.setPower(0.5);
                        sleep(3000);
                        motorIntake.setPower(0);
                    }
                    break;

                case FINISH:
                    motorIntake.setPower(0);  // Ensure intake is stopped
                    stopRobot();
                    break;
            }
        }
    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}