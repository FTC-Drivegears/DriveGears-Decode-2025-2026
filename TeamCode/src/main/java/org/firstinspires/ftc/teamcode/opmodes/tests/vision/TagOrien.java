//package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//@TeleOp
//public class TagOrien extends LinearOpMode {
//
//    final double TURN_GAIN = 0.1;
//    final double MAX_AUTO_TURN = 0.3;
//
//    private DcMotorEx frontLeftDrive = null;
//    private DcMotorEx frontRightDrive = null;
//    private DcMotorEx backLeftDrive = null;
//    private DcMotorEx backRightDrive = null;
//
//    private static final boolean USE_WEBCAM = true;
//    private static final int DESIRED_TAG_ID = -1;  // -1 = detect any tag
//
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection desiredTag = null;
//
//    @Override
//    public void runOpMode() {
//
//        boolean targetFound = false;
//        double turn = 0;
//
//        initAprilTag();
//
//        // Motors
//        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "lf");
//        frontRightDrive = hardwareMap.get(DcMotorEx.class, "rf");
//        backLeftDrive = hardwareMap.get(DcMotorEx.class, "lb");
//        backRightDrive = hardwareMap.get(DcMotorEx.class, "rb");
//
//        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
//        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
//        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
//
//        if (USE_WEBCAM)
//            setManualExposure(2, 230);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            targetFound = false;
//            desiredTag = null;
//
//            // Read tags
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    if (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID) {
//                        targetFound = true;
//                        desiredTag = detection;
//                        break;
//                    }
//                } else {
//                    telemetry.addData("Unknown", "Tag ID %d not in library", detection.id);
//                }
//            }
//
//            if (targetFound) {
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//            } else {
//                telemetry.addData("Info", "Drive to find a tag");
//            }
//
//            // Auto-turn when LB held
//            if (gamepad1.left_bumper && targetFound) {
//
//                double headingError = desiredTag.ftcPose.bearing;
//                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//
//                telemetry.addData("Auto Turn", "%5.2f", turn);
//
//            } else {
//                // Manual turn
//                turn = -gamepad1.right_stick_x / 3.0;
//                telemetry.addData("Manual Turn", "%5.2f", turn);
//            }
//
//            telemetry.update();
//
//            // FIXED: send turn as yaw
//            moveRobot(0, 0, turn);
//
//            sleep(10);
//        }
//    }
//
//    // Mecanum movement (x, y, yaw)
//    public void moveRobot(double x, double y, double yaw) {
//        double frontLeftPower = x - y - yaw;
//        double frontRightPower = x + y + yaw;
//        double backLeftPower = x + y - yaw;
//        double backRightPower = x - y + yaw;
//
//        double max = Math.max(1.0, Math.max(
//                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
//                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
//        ));
//
//        frontLeftPower /= max;
//        frontRightPower /= max;
//        backLeftPower /= max;
//        backRightPower /= max;
//
//        frontLeftDrive.setPower(frontLeftPower);
//        frontRightDrive.setPower(frontRightPower);
//        backLeftDrive.setPower(backLeftPower);
//        backRightDrive.setPower(backRightPower);
//    }
//
//    private void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(3);
//
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(aprilTag)
//                    .build();
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessor(aprilTag)
//                    .build();
//        }
//    }
//
//    private void setManualExposure(int exposureMS, int gain) {
//
//        if (visionPortal == null) return;
//
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting...");
//            telemetry.update();
//
//            while (!isStopRequested() &&
//                    visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//                sleep(20);
//            }
//        }
//
//        ExposureControl exposureControl =
//                visionPortal.getCameraControl(ExposureControl.class);
//
//        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//            exposureControl.setMode(ExposureControl.Mode.Manual);
//            sleep(50);
//        }
//
//        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
//        sleep(20);
//
//        GainControl gainControl =
//                visionPortal.getCameraControl(GainControl.class);
//
//        gainControl.setGain(gain);
//        sleep(20);
//    }
//}