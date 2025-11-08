package org.firstinspires.ftc.teamcode.opmodes.tests;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "AutoVision")
public class Obelisk_Auto extends LinearOpMode {
    private MecanumCommand mecanumCommand;

    private ElapsedTime resetTimer;

    enum AUTO_STATE {
        SCAN_OBELISK,
        PICKUP,
        FINISH
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
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

            processTelemetry();

            switch (autoState) {
                case SCAN_OBELISK:
                    if (mecanumCommand.moveToPos(0, 0,0)) {
                        if (tagProcessor.getDetections().size() > 0) {
                            AprilTagDetection tag = tagProcessor.getDetections().get(-1);
                            telemetry.addData("ID", tag.id);
                        }
                    }
                    telemetry.update();
                    if (mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.FINISH;
                    }
                    break;
                case PICKUP:
//                    if (mecanumCommand.moveToPos(30, -20, 0)) {
//                        autoState = AUTO_STATE.FINISH;
//                    }
//                    break;
                case FINISH:
                    stopRobot();
                    break;
            }
        }

    }
    public void processTelemetry(){
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Theta", mecanumCommand.getOdoHeading());
        telemetry.update();
    }
    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}


