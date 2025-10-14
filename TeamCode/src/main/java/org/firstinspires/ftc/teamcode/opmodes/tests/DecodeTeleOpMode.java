package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class DecodeTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private Hardware hw;
    private double theta;

    private DcMotor motorTurret;
    private DcMotor motorHood;

    private DcMotor motorIntake;

    private Servo servoSorter;
    private Servo servoPusher;

    private ElapsedTime resetTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        this.motorTurret = hardwareMap.get(DcMotorEx.class, "turret");
        this.motorHood = hardwareMap.get(DcMotorEx.class, "hood");
        motorTurret.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHood.setDirection(DcMotorSimple.Direction.FORWARD);

        this.motorIntake = hardwareMap.get(DcMotor.class, "externalIntake");
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        boolean isIntakeMotorOn = false;

        this.servoSorter = hardwareMap.get(Servo.class, "sorter");
        this.servoPusher = hardwareMap.get(Servo.class, "pusher");

        int buttonCounter = 0;
        boolean buttonCondition;
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        resetTimer = new ElapsedTime();

        double setServoSorterPosition = 0;
        double supposedServoSorterPosition = 0;

        servoSorter.setPosition(setServoSorterPosition);

        while (opModeInInit()){
            telemetry.update();
        }

        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                servoPusher.setPosition(1);
                motorTurret.setPower(gamepad1.right_trigger);
            }
            servoPusher.setPosition(0);
            if (gamepad1.left_bumper) {
                motorHood.setPower(-gamepad1.left_trigger);
            }
            else {
                motorHood.setPower(gamepad1.left_trigger);
            }

            if (gamepad1.x) {
                // Go to next sort
                Thread.sleep(1000);
                if (servoSorter.getPosition() == 0.3+setServoSorterPosition) {
                    supposedServoSorterPosition = 0.6;
                } else if (servoSorter.getPosition() == 0.6+setServoSorterPosition) {
                    supposedServoSorterPosition = 1;
                } else {
                    supposedServoSorterPosition = 0.3;
                }
            }

            if (gamepad1.y) {
                // Manual override
                setServoSorterPosition += 0.05;
            }

            setServoSorterPosition = (setServoSorterPosition+supposedServoSorterPosition)%1;
            servoSorter.setPosition(setServoSorterPosition);
            // No longer needs correction

            // Intake
            theta = mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (gamepad1.right_bumper) {
                isIntakeMotorOn = !isIntakeMotorOn;
                telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
                if (isIntakeMotorOn){
                    motorIntake.setPower(0.8);
                }else{
                    motorIntake.setPower(0);
                }
            }

            if (gamepad1.a) {
                motorIntake.setPower(-0.3);
            } else {
                motorTurret.setPower(0);
                motorIntake.setPower(0);
            }
        }
    }
    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");

        telemetry.update();
    }
}
