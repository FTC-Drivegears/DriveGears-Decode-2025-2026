package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class LimelightAprilTag extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);//april tag pipeline
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llResult = limelight.getLatestResult();
//
//        if (llResult != null && llResult.isValid()) {
//            Pose3D botPose = llResult.getBotpose_MT2();
//            //Pose3D botPose = llResult.getBotpose();
//            telemetry.addData("Tx", llResult.getTx());
//            telemetry.addData("Ty", llResult.getTy());
//            telemetry.addData("Ta", llResult.getTa());
//
//            // check fiducials (AprilTags)
//            if (llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
//                // show the first detected tag ID
//                int tagId = llResult.getFiducialResults().get(0).getFiducialId();
//                telemetry.addData("Detected Tag ID", tagId);
//            } else {
//                telemetry.addLine("No AprilTags detected");
//            }
//        }
//        telemetry.update();
//    }
//}

LLStatus status = limelight.getStatus();
       telemetry.addData("Name", "%s",
                         status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                          status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                          status.getPipelineIndex(), status.getPipelineType());

LLResult result = limelight.getLatestResult();
       if (result != null && result.isValid()) {
double tx = result.getTx(); // How far left or right the target is (degrees)
double ty = result.getTy(); // How far up or down the target is (degrees)

           telemetry.addData("Target X", tx);
           telemetry.addData("Target Y", ty);
       } else {
               telemetry.addData("Limelight", "No Targets");
       }
               }
               }
