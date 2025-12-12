package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class LimelightTurret extends LinearOpMode {
    private DcMotorEx llmotor;
    private Limelight3A limelight;
    private Hardware hw;
    private String ALLIANCE;
    public String OBELISK = "UNKNOWN";
    private static int targetID = -1;
    private static double x_Value = Double.NaN;
    private static AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llmotor = hardwareMap.get(DcMotorEx.class, "llmotor");
        limelight.pipelineSwitch(7);
        limelight.start();

        telemetry.addLine("waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Name", status.getName());
            telemetry.addData("CPU", "%.1f %%", status.getCpu());
            telemetry.addData("FPS", "%d", (int) status.getFps());
            telemetry.addData("Pipeline", "%d (%s)",
                    status.getPipelineIndex(),
                    status.getPipelineType()
            );

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                if (tx > 3) {
                    llmotor.setPower(-0.5);
                } else if (tx < -3) {
                    llmotor.setPower(0.5);
                } else {
                    llmotor.setPower(0);
                }
                telemetry.addData("tx", tx);
                telemetry.update();

            } else {
                if (gamepad2.dpad_right) {
                    llmotor.setPower(-0.5);
                } else if (gamepad2.dpad_left) {
                    llmotor.setPower(0.5);
                } else {
                    llmotor.setPower(0);
                }
            }
        }

    }
}