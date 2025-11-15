package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LogitechVisionSubsystem {
    private final Hardware hw;
    private final CameraName Webcam_1;
    private String ALLIANCE;
    public String OBELISK = "UNKNOWN";
    private int targetID = -1;
    private double x_Value = Double.NaN;
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public LogitechVisionSubsystem(Hardware hw, String alliance) {
        this.hw = hw;
        this.Webcam_1 = hw.Webcam_1;
        this.ALLIANCE = alliance;

        if (ALLIANCE != null && "RED".equalsIgnoreCase(alliance)) {
            this.targetID = 24;
        } else if (ALLIANCE != null && "BLUE".equalsIgnoreCase(alliance)) {
            this.targetID = 20;
        } else {
            this.targetID = -1;
        }

        this.tagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(this.Webcam_1);
        builder.setAutoStopLiveView(true);
        builder.addProcessor(this.tagProcessor);
        this.visionPortal = builder.build();
    }

    public String pattern() {
        if (tagProcessor == null) return OBELISK;
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        if (currentDetections == null || currentDetections.isEmpty()) return OBELISK;

        for (AprilTagDetection detection : currentDetections) {
            if (detection != null && detection.metadata != null) {
                int id = detection.id;
                if (id == 21 || id == 22 || id == 23) {
                    OBELISK = detection.metadata.name;
                    return OBELISK;
                }
            }
        }
        return OBELISK;
    }

    public double targetApril(Telemetry telemetry) {
        if (tagProcessor == null) return Double.NaN;
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        if (currentDetections == null) return Double.NaN;

        for (AprilTagDetection detection : currentDetections) {
            if (detection != null && detection.metadata != null && detection.id == this.targetID) {
                if (telemetry != null) {
                    telemetry.addData("April tag height", detection.ftcPose.z);
                    telemetry.addData("April tag angle", detection.ftcPose.yaw);
                    telemetry.addData("April tag id", this.targetID);
                }
                this.x_Value = detection.ftcPose.x;
                return this.x_Value;
            }
        }
        return Double.NaN;
    }

    public double targetApril() {
        return targetApril(null);
    }

    public void telemetryAprilTag(Telemetry telemetry) {
        if (tagProcessor == null) {
            if (telemetry != null) telemetry.addData("AprilTag", "tagProcessor is null");
            return;
        }

        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        int count = (currentDetections == null) ? 0 : currentDetections.size();
        if (telemetry != null) telemetry.addData("# AprilTags Detected", count);

        if (currentDetections == null || currentDetections.isEmpty()) {
            if (telemetry != null) telemetry.addLine("No detection");
            return;
        }

        for (AprilTagDetection detection : currentDetections) {
            if (detection != null && detection.metadata != null) {
                if (telemetry != null) {
                    telemetry.addLine(String.format("==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("%s", detection.metadata.name));
                }
            } else {
                if (telemetry != null) telemetry.addLine("No detection");
            }
        }
    }
}
