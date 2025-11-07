package org.firstinspires.ftc.teamcode.opmodes.tests;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private int stage1 = 0;

    private ElapsedTime resetTimer;

    enum AUTO_STATE {
        SCAN_OBELISK,
        SUB_PICKUP,
        FINISH

    }

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        resetTimer = new ElapsedTime();

        AUTO_STATE autoState = AUTO_STATE.SCAN_OBELISK;
        waitForStart();

        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();
            updateTelemetry();

            switch (autoState) {
                case SCAN_OBELISK:
                    if (tagProcessor.getDetections().size() > 0) {
                        AprilTagDetection tag = tagProcessor.getDetections().get(0);
                        mecanumCommand.moveToPos(0, 0, 0);

                    if (mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.FINISH;
                    }
                    break;
                case FINISH:
                    stopRobot();
                    break;
            }
        }
    }
    public void updateTelemetry () {
        telemetry.addData("ID", tag.id);
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("Theta: ", mecanumCommand.getOdoHeading());
        telemetry.update();
    }
    private void stopRobot() {
        mecanumCommand.stop();
    }
}
