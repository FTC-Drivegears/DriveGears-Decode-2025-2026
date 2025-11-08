package org.firstinspires.ftc.teamcode.opmodes.tests.competition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class DecodeTeleOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private Hardware hw;
    private double theta;
    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private Servo hood;
    private SorterSubsystem sorterSubsystem;
    private long lastIntakeTime;
    private long lastFireTime;
    private long lastOuttakeTime;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousXState = false;
        boolean previousYState = false;
        boolean prevRightTrigger = false;
        boolean prevLeftTrigger = false;
        boolean prevRB = false;
        boolean prevLB = false;

        boolean currentXState;
        boolean currentYState;
        boolean curRightTrigger;
        boolean curLeftTrigger;
        boolean curRB;
        boolean curLB;

        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean togglePusher = false;
        boolean toggleOuttakeSorter = false;

        double hoodPos = 0.0;


        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        pusher = hw.pusher;
        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);

        hw.sorter.setPosition(0.0);
        hw.hood.setPosition(1.0);

        intake = hw.intake;
        shooter = hw.shooter;
        hood = hw.hood;

        intake.setDirection(DcMotorSimple.Direction.REVERSE);


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

            curRightTrigger = gamepad1.right_trigger > 0;
            if (curRightTrigger && !prevRightTrigger){
                isIntakeMotorOn = !isIntakeMotorOn;

                if (isIntakeMotorOn){
                    intake.setPower(0.65);
                }else {
                    intake.setPower(0);
                }
            }
            prevRightTrigger = curRightTrigger;

            curLeftTrigger = gamepad1.left_trigger > 0;
            if (curLeftTrigger && !prevLeftTrigger){ // Press left to outtake;
                toggleOuttakeSorter = !toggleOuttakeSorter;

                if (toggleOuttakeSorter){
                    double durationOuttake = (System.nanoTime() - lastOuttakeTime)/1E9;
                    if (durationOuttake >= 1) {
                        telemetry.addLine("outtake");
                        //sorterSubsystem.outtakeBall();
                        lastOuttakeTime = System.nanoTime();
                        telemetry.update();
                    }
                }
            }

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
                    telemetry.update();
                }
            }
            if (gamepad1.a){ // Press A to quick fire.
                double durationFire = (System.nanoTime() - lastFireTime)/1E9;
                if (durationFire >= 1) {
                    telemetry.addLine("quick firing");
                    sorterSubsystem.quickFire();
                    lastFireTime = System.nanoTime();
                    telemetry.update();
                }
            }

            currentYState = gamepad1.y;
            if (currentYState && !previousYState){
                togglePusher = !togglePusher;

                if (togglePusher){
                    pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
                    sorterSubsystem.setIsPusherUp(true);
                }else{
                    pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
                    sorterSubsystem.setIsPusherUp(false);
                }
            }
            previousYState = currentYState;

            currentXState = gamepad1.x;
            if (currentXState && !previousXState){
                isOuttakeMotorOn = !isOuttakeMotorOn;

                if (isOuttakeMotorOn){
                    shooter.setPower(0.8);
                }else{
                    shooter.setPower(0);
                }
            }
            previousXState = currentXState;

            curRB = gamepad1.right_bumper;
            if(curRB){
                if(hoodPos >= 1.0){
                    hoodPos = 1.0;
                }
                else{
                    hoodPos += 0.001;
                }
                hood.setPosition(hoodPos);
            }

            curLB = gamepad1.left_bumper;
            if(curLB){
                if(hoodPos <= 0.0){
                    hoodPos = 0.0;
                }
                else{
                    hoodPos -= 0.001;
                }
                hood.setPosition(hoodPos);
            }

            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
            telemetry.addData("Hood pos: ", hoodPos);
            telemetry.update();
        }
    }
}