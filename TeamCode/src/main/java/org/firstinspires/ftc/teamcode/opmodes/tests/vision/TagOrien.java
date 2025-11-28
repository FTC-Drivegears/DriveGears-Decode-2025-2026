package org.firstinspires.ftc.teamcode.opmodes.tests.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@TeleOp
public class TurnOrien extends LinearOpMode {
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    final double TURN_GAIN = 0.5;
    final double MAX_AUTO_TURN = 0.3; //max speed
    private double previousTurn = 0; //last turn value

    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx backRightDrive;
    private LogitechVisionSubsystem vision;

    @Override
    public void runOpMode() {

        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        vision = new LogitechVisionSubsystem(hw, "BLUE"); // or red

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "rf");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "lb");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "rb");

        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            Double headingError = vision.getTargetYaw();
            boolean targetFound = (headingError != null);

            double turn = 0;

            if (gamepad1.left_bumper && targetFound) {

                double error = headingError;

                double deadzone = 2.0; //tune
                if (Math.abs(error) < deadzone) {
                    turn = 0;
                    previousTurn = 0;
                    telemetry.addLine("ALIGNED");

                } else {
                    double actualTurn = error * TURN_GAIN;
                    actualTurn = Range.clip(actualTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    double SMOOTHING = 0.6;
                    turn = actualTurn * (1 - SMOOTHING) + previousTurn * SMOOTHING;
                    previousTurn = turn;

                    telemetry.addData("AUTO TURN", "%.2f", turn);
                    telemetry.addData("Yaw Error", "%.2f°", error);
                }

            } else {
                turn = -gamepad1.right_stick_x / 3.0;
                previousTurn = turn;
                telemetry.addData("MANUAL TURN", "%.2f", turn);
            }

            telemetry.update();
            moveRobot(0, 0, turn);

            sleep(10);
        }
    }
    public void moveRobot(double x, double y, double yaw) {
        double fl = x - y - yaw;
        double fr = x + y + yaw;
        double bl = x + y - yaw;
        double br = x - y + yaw;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }
}
