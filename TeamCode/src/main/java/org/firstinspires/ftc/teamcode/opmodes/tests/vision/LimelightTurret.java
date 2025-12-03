package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LimelightTurret extends OpMode {
    private DcMotorEx llmotor;
    private Limelight3A limelight;

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llmotor = hardwareMap.get(DcMotorEx.class, "llmotor");
        limelight.pipelineSwitch(7);
    }

    public void start() {
        limelight.start();
    }

    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // how left or right the target is (degrees)
            double ty = result.getTy(); // how  up or down the target is (degrees)

            if (tx > 5) {
                llmotor.setPower(0.3);
            } else if (tx < -5) {
                llmotor.setPower(-0.3);
            } else {
                llmotor.setPower(0);
            }
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}

