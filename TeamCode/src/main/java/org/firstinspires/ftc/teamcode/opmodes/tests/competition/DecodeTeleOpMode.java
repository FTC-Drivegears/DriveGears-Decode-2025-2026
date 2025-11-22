package org.firstinspires.ftc.teamcode.opmodes.tests.competition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
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
    private Servo hood;
    private SorterSubsystem sorterSubsystem;
    private long lastFireTime;
    private long lastIntakeTime;
    private long lastOuttakeTime;

    boolean isManualPushOn;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousXState = false;
        boolean previousYState = false;
        boolean prevRightTrigger = false;
        boolean prevLeftTrigger = false;
        boolean currentXState;
        boolean currentYState;
        boolean curRightTrigger;
        boolean curLeftTrigger;
        boolean curRB;
        boolean curLB;
        boolean shouldSpinSorter;

        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean togglePusher = false;
        boolean toggleOuttakeSorter = false;

        double hoodPos = 0.846;
        double shootSpeed = 4000;

        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hw);
        Servo pusher = hw.pusher;
        Servo light = hw.light;
        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.light.setPosition(0.0);
        hw.sorter.setPosition(0.1);
        hw.hood.setPosition(hoodPos);

        intake = hw.intake;
        shooter = hw.shooter;
        hood = hw.hood;

        if (sorterSubsystem == null) { // sorterSubsystem is only set once
            sorterSubsystem = new SorterSubsystem(hw,this, telemetry, "");
        }

        while (opModeInInit()){
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            mecanumCommand.processOdometry();
            theta = mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            // Manually spin sorter plate.

            boolean shouldIntakeGreen = gamepad1.dpad_left;
            boolean shouldIntakePurple = gamepad1.dpad_right;
            if (shouldIntakeGreen || shouldIntakePurple) {
                double durationIntake = (System.nanoTime() - lastIntakeTime)/1E9;
                char curColor = 'g';
                if (shouldIntakePurple) {
                    curColor = 'p';
                }
                if (durationIntake >= 0.7) {
                    sorterSubsystem.intakeBall(curColor);
                    lastIntakeTime = System.nanoTime();
                }
            }

            // Manually outtake ball.
            boolean shouldOuttakePurple = gamepad1.left_trigger > 0;
            boolean shouldOuttakeGreen = gamepad1.b;
            if (shouldOuttakePurple || shouldOuttakeGreen) {
                double durationOuttake = (System.nanoTime() - lastOuttakeTime)/1E9;
                char curColor = 'g';
                if (shouldOuttakePurple) {
                    curColor = 'p';
                }
                if (durationOuttake >= 0.7) {
                    sorterSubsystem.outtakeBall(curColor);
                    isManualPushOn = false;
                    lastOuttakeTime = System.nanoTime();
                }
            }
            if (gamepad1.a){ // Press A to quick fire.
                double durationFire = (System.nanoTime() - lastFireTime)/1E9;
                if (durationFire >= 0.7) {
                    sorterSubsystem.quickFire();
                    isManualPushOn = false;
                    lastFireTime = System.nanoTime();
                }
            }

            if (!isManualPushOn) {
                sorterSubsystem.pushDown(); // Will push down if the pusher is up by outtake.
            }

            curRightTrigger = gamepad1.right_trigger > 0;
            if (curRightTrigger && !prevRightTrigger){
                isIntakeMotorOn = !isIntakeMotorOn;

                if (isIntakeMotorOn){
                    intake.setPower(-0.8);
                }else {
                    intake.setPower(0);
                }
            }
            prevRightTrigger = curRightTrigger;




            currentYState = gamepad1.y;
            if (currentYState && !previousYState){
                togglePusher = !togglePusher;

                if (togglePusher){
                    isManualPushOn = true;
                    pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
                    sorterSubsystem.setIsPusherUp(true);
                } else {
                    pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
                    sorterSubsystem.setIsPusherUp(false);
                }
            }
            previousYState = currentYState;

            currentXState = gamepad1.x;
            if (currentXState && !previousXState){
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;


            curRB = gamepad2.right_bumper;
            if(curRB){
                if(hoodPos <= 0.359){
                    hoodPos = 0.359;
                }
                else{
                    hoodPos -= 0.001;
                }
                hood.setPosition(hoodPos);
            }

            curLB = gamepad2.left_bumper;
            if(curLB){
                if(hoodPos >= 0.846){
                    hoodPos = 0.846;
                }
                else{
                    hoodPos += 0.001;
                }
                hood.setPosition(hoodPos);
            }

            if(gamepad2.dpad_up){
                if(shootSpeed >= 6000.0){
                    shootSpeed = 6000.0;
                }
                else {
//                    shootSpeed += 0.0001;
                    shootSpeed += 150.0;
                    sleep(500);
                }
            }

            if(gamepad2.dpad_down) {
                if (shootSpeed <= 0.0) {
                    shootSpeed = 0.0;
                } else {
//                    shootSpeed -= 0.0001;
                    shootSpeed -= 150.0;
                    sleep(500);
//0.8 default shooter speed
                }
            }

            if (gamepad1.start){
                mecanumCommand.resetPinPointOdometry();
            }

            if (isOuttakeMotorOn){
                shooterSubsystem.setMaxRPM(shootSpeed);
                if (shooterSubsystem.spinup()){
                    light.setPosition(1.0);
                } else {
                    light.setPosition(0.0);
                }

            }else{
                shooterSubsystem.stopShooter();
                light.setPosition(0.0);
            }

//            telemetry.addData("Hood pos: ", hoodPos);
//            telemetry.addLine("---------------------------------");
//            telemetry.addData("X", mecanumCommand.getX());
//            telemetry.addData("Y", mecanumCommand.getY());
//            telemetry.addData("Theta", mecanumCommand.getOdoHeading());
//            telemetry.addData("Outtake speed: ", shootSpeed);
            telemetry.update();
        }
    }
}