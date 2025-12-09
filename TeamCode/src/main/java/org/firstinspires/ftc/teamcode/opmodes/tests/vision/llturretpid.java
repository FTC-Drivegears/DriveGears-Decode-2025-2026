package org.firstinspires.ftc.teamcode.opmodes.tests.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class llturretpid extends LinearOpMode {

    private DcMotorEx llmotor;
    private Limelight3A limelight;


    private double kP = 0.02;
    private double kD = 0.0005;

    private double lastError = 0;
    private double lastTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llmotor = hardwareMap.get(DcMotorEx.class, "llmotor");

        limelight.pipelineSwitch(7);
        limelight.start();

        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();

        lastTime = getRuntime();

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

                double tx = result.getTx(); // want tx → 0

                double error = tx;
                double currentTime = getRuntime();
                double dt = currentTime - lastTime;

                double derivative = (error - lastError) / dt;

                double output = (kP * error) + (kD * derivative);

                output = Math.max(Math.min(output, -0.5), 0.5);

                llmotor.setPower(output);

                lastError = error;
                lastTime = currentTime;

                telemetry.addData("tx", tx);
                telemetry.addData("PD Output", output);
                telemetry.update();

            } else {
                llmotor.setPower(0);

                if (gamepad2.left_trigger > 0) {
                    llmotor.setPower(-0.5);
                } else if (gamepad2.right_trigger > 0) {
                    llmotor.setPower(0.5);
                } else {
                    llmotor.setPower(0);
                }
            }
        }
    }
}