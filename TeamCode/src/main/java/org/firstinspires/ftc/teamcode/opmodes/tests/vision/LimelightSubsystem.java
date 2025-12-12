package org.firstinspires.ftc.teamcode.opmodes.tests.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LimelightSubsystem {
    private DcMotorEx llmotor;
    private Limelight3A limelight;
    private Hardware hw;
    private String ALLIANCE;
    public String OBELISK = "UNKNOWN";
    private static int targetID = -1;
    private static double x_Value = Double.NaN;
    private static AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public LimelightSubsystem (Hardware hw, String alliance) {
        this.hw = hw;
        this.limelight = hw.limelight;
        this.ALLIANCE = alliance;
        limelight.pipelineSwitch(7);
        limelight.start();

        if (ALLIANCE != null && "RED".equalsIgnoreCase(alliance)) {
            this.targetID = 24;
        } else if (ALLIANCE != null && "BLUE".equalsIgnoreCase(alliance)) {
            this.targetID = 20;
        } else {
            this.targetID = -1;
        }
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





}
