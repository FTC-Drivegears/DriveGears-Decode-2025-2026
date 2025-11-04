package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class DecodeTeleOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;



    private Hardware hw;
    private double theta;
    private DcMotor intake;
    private DcMotor outtake;

    private Servo pusher;

    private SorterSubsystem sorterSubsystem;

    private long lastIntakeTime;
    private long lastFireTime;
    private long lastOuttakeTime;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousAState = false;
        boolean previousXState = false;
        boolean previousYState = false;
        boolean currentAState;
        boolean currentXState;
        boolean currentYState;

        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean togglePusher = false;


        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        pusher = hw.pusher;
        intake = hw.intake;
        outtake = hw.intake;

        // pusher.setPosition(1);
        if (sorterSubsystem == null) { // sorterSubsystem is only set once
            sorterSubsystem = new SorterSubsystem(hw,this, telemetry, "pgg");
        }

        while (opModeInInit()){
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            theta = mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            currentAState = gamepad1.a;
            if (currentAState && !previousAState){
                isIntakeMotorOn = !isIntakeMotorOn;

                if (isIntakeMotorOn){
                    intake.setPower(0.8);
                }else {
                    intake.setPower(0);
                }
            }
            previousAState = currentAState;

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            if (up || down) { // Press up to intake g, down to intake p.
                double durationIntake = (System.nanoTime() - lastIntakeTime)/1E9;
                char curColor = 'g';
                if (down) {
                    curColor = 'p';
                }
                if (durationIntake >= 2) {
                    telemetry.addData("mockInputBall", curColor);
                    sorterSubsystem.intakeBall(curColor);
                    lastIntakeTime = System.nanoTime();
                }
            }
            if (gamepad1.dpad_right){ // Press right to quick fire.
                double durationFire = (System.nanoTime() - lastFireTime)/1E9;
                if (durationFire >= 1) {
                    telemetry.addLine("quick firing");
                    sorterSubsystem.quickFire();
                    lastFireTime = System.nanoTime();
                }
            }
            if (gamepad1.dpad_left){ // Press left to outtake;
                double durationOuttake = (System.nanoTime() - lastOuttakeTime)/1E9;
                if (durationOuttake >= 1) {
                    telemetry.addLine("outtake");
                    sorterSubsystem.outtakeBall();
                    lastOuttakeTime = System.nanoTime();
                }
            }

            currentYState = gamepad1.y;
            if (currentYState && !previousYState){
                togglePusher = !togglePusher;

                if (togglePusher){
                    pusher.setPosition(0);
                }else{
                    pusher.setPosition(1);
                }
            }
            previousYState = currentYState;

            currentXState = gamepad1.x;
            if (currentXState && !previousXState){
                isOuttakeMotorOn = !isOuttakeMotorOn;

                if (isOuttakeMotorOn){
                    outtake.setPower(1);
                }else{
                    outtake.setPower(0);
                }
            }
            previousXState = currentXState;

            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
            telemetry.update();
        }
    }
}