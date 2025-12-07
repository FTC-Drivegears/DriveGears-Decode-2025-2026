package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LimelightTurret extends LinearOpMode {
    private DcMotorEx llmotor;
    private Limelight3A limelight;

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
                if (tx > 5) {
                    llmotor.setPower(-0.4);
                } else if (tx < -5) {
                    llmotor.setPower(0.4);
                } else {
                    llmotor.setPower(0);
                }

                telemetry.addData("tx", tx);
                telemetry.update();

            } else {
                llmotor.setPower(0);
                if (gamepad2.left_trigger > 0) {
                    llmotor.setPower(1);
                } else {
                    llmotor.setPower(0);
                }

                if (gamepad2.right_trigger > 0) {
                    llmotor.setPower(-1);
                } else {
                    llmotor.setPower(0);
                }
            }
        }
    }
}