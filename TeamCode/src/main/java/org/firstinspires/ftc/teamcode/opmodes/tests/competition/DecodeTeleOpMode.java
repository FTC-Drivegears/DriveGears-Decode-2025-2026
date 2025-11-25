package org.firstinspires.ftc.teamcode.opmodes.tests.competition;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class DecodeTeleOpMode extends LinearOpMode {
    private Hardware hw;
    private IMU imu;
    private double theta;
    private DcMotor intake;
    private DcMotor shooter;
    private Servo hood;
    private SorterSubsystem sorterSubsystem;
    private long lastFireTime;
    private long lastIntakeTime;

    private long lastOuttakeTime;


    //Shooter Presets
    private final double FAR_HOOD = 0.5;
    private final int FAR_SHOOT_SPEED = 3900;
    private final double MID_HOOD = 0.6;
    private final int MID_SHOOT_SPEED = 3000;
    private final double CLOSE_HOOD = 0.846;
    private final int CLOSE_SHOOT_SPEED = 2500;

    private final ElapsedTime sorterTimer = new ElapsedTime();

    double sorterPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumCommand mecanumCommand = new MecanumCommand(hw);
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hw);
        IMU imu = hw.imu;
        hw = Hardware.getInstance(hardwareMap);
        DcMotorEx lf = hw.lf;
        DcMotorEx lb = hw.lb;
        DcMotorEx rf = hw.rf;
        DcMotorEx rb = hw.rb;

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        rb.setDirection(DcMotorEx.Direction.REVERSE);

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

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, //what the orientation of the logo on the REV HUB
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));//what the orientation of usb is
                //these might not be correct
        imu.initialize(parameters);

        while (opModeInInit()){
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.start){
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX*1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double lfPower = (rotY + rotX + rx) / denominator;
            double lbPower = (rotY - rotX + rx) / denominator;
            double rfPower = (rotY - rotX - rx) / denominator;
            double rbPower = (rotY + rotX - rx) / denominator;

            lf.setPower(lfPower);
            lb.setPower(lbPower);
            rf.setPower(rfPower);
            rb.setPower(rbPower);

            // Manually spin sorter plate.

            boolean shouldIntakeGreen = gamepad2.dpad_down;
            boolean shouldIntakePurple = gamepad2.dpad_up;
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
            boolean shouldOuttakePurple = gamepad1.dpad_up;
            boolean shouldOuttakeGreen = gamepad1.dpad_down;
            if (shouldOuttakePurple || shouldOuttakeGreen) {
                double durationOuttake = (System.nanoTime() - lastOuttakeTime)/1E9;
                char curColor = 'g';
                if (shouldOuttakePurple) {
                    curColor = 'p';
                }
                if (durationOuttake >= 0.7) {
                    sorterSubsystem.outtakeBall(curColor);
                    lastOuttakeTime = System.nanoTime();
                }
            }
            if (gamepad1.a){ // Press A to quick fire.
                double durationFire = (System.nanoTime() - lastFireTime)/1E9;
                if (durationFire >= 0.7) {
                    sorterSubsystem.quickFire();
                    lastFireTime = System.nanoTime();
                }
            }
            sorterSubsystem.pushDown(); // Will push down if the pusher is up.

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


            if (gamepad2.b && sorterTimer.milliseconds() > 1000){
                sorterPosition = (sorterPosition+1)%3;
                sorterTimer.reset();
                if (sorterPosition == 0) {
                    hw.sorter.setPosition(0.085);//60 degrees
                } else if (sorterPosition == 1) {
                    hw.sorter.setPosition(0.515);//60 degrees
                } else if (sorterPosition == 2) {
                    hw.sorter.setPosition(0.96);//60 degrees
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
            }
            previousXState = currentXState;

            // CLOSE
            if (gamepad2.a) {
                hood.setPosition(CLOSE_HOOD);
                shootSpeed = CLOSE_SHOOT_SPEED;
            }
            //MID
            if (gamepad2.x) {
                hood.setPosition(MID_HOOD);
                shootSpeed = MID_SHOOT_SPEED;
            }
            //FAR
            if (gamepad2.y){
                hood.setPosition(FAR_HOOD);
                shootSpeed = FAR_SHOOT_SPEED;
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