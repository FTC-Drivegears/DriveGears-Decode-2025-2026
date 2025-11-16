package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@TeleOp
public class AprilTagTeleOp extends LinearOpMode {
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private LogitechVisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {

        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        vision = new LogitechVisionSubsystem(hw, "BLUE");

        waitForStart();

        double maxPower = 1;
        double deadZone = 0.0; //inches

        while (opModeIsActive()) {

            boolean alignPressed = gamepad2.b;

            mecanumCommand.fieldOrientedMove(
                    gamepad2.left_stick_y,
                    gamepad2.left_stick_x,
                    gamepad2.right_stick_x
            );

            if (alignPressed) {

                double xOffset = vision.targetApril(telemetry); //inches
                telemetry.addData("ftcPose.x", xOffset);

                if (!Double.isNaN(xOffset)) {

                    double kP = (Math.abs(xOffset) < 3.0) ? 0.008 : 0.015;

                    if (Math.abs(xOffset) > deadZone) {

                        // More offset = more turning power
                        double turnPower = -xOffset * kP;

                        turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

                        mecanumCommand.fieldOrientedMove(0, 0, turnPower);

                        telemetry.addData("Turn Power", turnPower);

                    } else {
                        mecanumCommand.fieldOrientedMove(0, 0, 0);
                        telemetry.addLine("Centered");
                    }

                } else {
                    telemetry.addLine("No Tag Detected");
                }
            }
            telemetry.update();
        }
    }
}
