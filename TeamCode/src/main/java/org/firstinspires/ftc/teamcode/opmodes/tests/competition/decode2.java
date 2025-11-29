package org.firstinspires.ftc.teamcode.opmodes.tests.competition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.opmodes.tests.vision.LogitechVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class decode2 extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private Hardware hw;
    private double theta;
    final double TURN_GAIN = 0.5;
    final double MAX_AUTO_TURN = 0.3; //max speed
    private double previousTurn = 0; //last turn value
    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private Servo hood;
    private Servo light;

    private SorterSubsystem sorterSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private final ElapsedTime sorterTimer = new ElapsedTime();

    private final ElapsedTime outtakeTimer = new ElapsedTime();

    private final ElapsedTime pusherTimer = new ElapsedTime();

    private final ElapsedTime colorSensingTimer = new ElapsedTime();

    private final ElapsedTime tagOrientTimer = new ElapsedTime();

    //Shooter Presets
    private final double FAR_HOOD = 0.4;
    private final int FAR_SHOOT_SPEED = 3700;
    private final double MID_HOOD = 0.6;
    private final int MID_SHOOT_SPEED = 3050;
    private final double CLOSE_HOOD = 0.846;
    private final int CLOSE_SHOOT_SPEED = 2500;

    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx backRightDrive;
    private LogitechVisionSubsystem vision;

    private enum DRIVETYPE {
        ROBOTORIENTED, FIELDORIENTED
    }

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousXState = false;
        boolean previousYState = false;

        boolean currentXState;
        boolean currentYState;

        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean rightTriggerPressed = false;
        boolean leftTriggerPressed = false;

        double hoodPos = 0.846;
        double shootSpeed = 4000;

        DRIVETYPE drivetype = DRIVETYPE.ROBOTORIENTED;

        hw = Hardware.getInstance(hardwareMap);


        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        vision = new LogitechVisionSubsystem(hw, "BLUE");
        pusher = hw.pusher;
        light = hw.light;
        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.light.setPosition(0.0);
        hw.sorter.setPosition(0.085);
        hw.hood.setPosition(hoodPos);

        intake = hw.intake;
        shooter = hw.shooter;
        hood = hw.hood;

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        if (sorterSubsystem == null) { // sorterSubsystem is only set once
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "");
        }

        while (opModeInInit()) {
            if (gamepad1.a) {
                drivetype = DRIVETYPE.FIELDORIENTED;
            }

            if (gamepad1.b) {
                drivetype = DRIVETYPE.ROBOTORIENTED;
            }

            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            Double headingError = vision.getTargetYaw();
            boolean targetFound = (headingError != null);

            double turn = 0;

            if (gamepad2.left_trigger > 0) {
                if (targetFound) {

                    double error = headingError;
                    double deadzone = 1.0;

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

                        telemetry.addData("Auto turn", "%.2f", turn);
                        telemetry.addData("Yaw Error", "%.2f°", error);
                    }

                } else {
                    turn = -gamepad1.right_stick_x / 3.0;
                    previousTurn = turn;
                    telemetry.addData("Turn manually", "%.2f", turn);
                }

                telemetry.update();
                mecanumCommand.robotOrientedMove(0, 0, turn);
                tagOrientTimer.reset();

            } else {
                mecanumCommand.robotOrientedMove(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                );
            }

            mecanumCommand.processOdometry();

//            if (drivetype == DRIVETYPE.FIELDORIENTED) {
//                theta = mecanumCommand.fieldOrientedMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            } else if (drivetype == DRIVETYPE.ROBOTORIENTED) {
//                theta = mecanumCommand.robotOrientedMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            }

            if (gamepad1.right_trigger > 0) {
                if (!rightTriggerPressed) {
                    rightTriggerPressed = true;
                    isIntakeMotorOn = !isIntakeMotorOn;
                    if (isIntakeMotorOn)
                        intake.setPower(0.8);
                    else
                        intake.setPower(0);
                }
            } else
                rightTriggerPressed = false;

            if (gamepad1.left_trigger > 0) {
                if (!leftTriggerPressed) {
                    leftTriggerPressed = true;
                    isIntakeMotorOn = !isIntakeMotorOn;
                    if (isIntakeMotorOn)
                        intake.setPower(-0.8);
                    else
                        intake.setPower(0);
                }
            } else
                leftTriggerPressed = false;


            boolean right = gamepad1.dpad_right;
            boolean left = gamepad1.dpad_left;
            if (right || left) { // right to spin sorter to green for outtake, left to spin sorter to purple for outtake
                if (outtakeTimer.milliseconds() > 500) {
                    char curColor = 'g';
                    if (left) {
                        curColor = 'p';
                    }
                    sorterSubsystem.outtakeBall(curColor);
                    outtakeTimer.reset();
                }
            }

            if (gamepad1.dpad_down && sorterTimer.milliseconds() > 1000) {
                sorterSubsystem.manualSpin();
                sorterTimer.reset();
            }

            if (colorSensingTimer.milliseconds() > 200) {
                sorterSubsystem.detectColor();
                colorSensingTimer.reset();
            }

            currentYState = gamepad1.y;
            if (currentYState && !previousYState) {
                // Start pulse only if not already pulsing
                if (!sorterSubsystem.getIsPusherUp()) {
                    sorterSubsystem.setIsPusherUp(true);
                    hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
                    pusherTimer.reset();
                }
            }
            previousYState = currentYState;

            // Pusher
            if (sorterSubsystem.getIsPusherUp() && pusherTimer.milliseconds() >= 500) {
                hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
                sorterSubsystem.setIsPusherUp(false);
            }

            currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;

            // CLOSE
            if (gamepad2.x) {
                hood.setPosition(CLOSE_HOOD);
                shootSpeed = CLOSE_SHOOT_SPEED;

            }
            //MID
            if (gamepad2.y) {
                hood.setPosition(MID_HOOD);
                shootSpeed = MID_SHOOT_SPEED;

               // public void moveRobot (double x, double y, double yaw) {

//        double fl = x - y - yaw;
//        double fr = x + y + yaw;
//        double bl = x + y - yaw;
//        double br = x - y + yaw;
//
//        double max = Math.max(1.0,
//                Math.max(Math.abs(fl),
//                        Math.max(Math.abs(fr),
//                                Math.max(Math.abs(bl), Math.abs(br)))));
//
//        frontLeftDrive.setPower(fl / max);
//        frontRightDrive.setPower(fr / max);
//        backLeftDrive.setPower(bl / max);
//        backRightDrive.setPower(br / max);
//    }
                }

                //FAR
                if (gamepad2.b) {
                    hood.setPosition(FAR_HOOD);
                    shootSpeed = FAR_SHOOT_SPEED;
                }

                if (gamepad1.start) {
                    mecanumCommand.resetPinPointOdometry();
                }

                if (isOuttakeMotorOn) {
                    shooterSubsystem.setMaxRPM(shootSpeed);
                    if (shooterSubsystem.spinup()) {
                        light.setPosition(1.0);
                    } else {
                        light.setPosition(0.0);
                    }

                } else {
                    shooterSubsystem.stopShooter();
                    light.setPosition(0.0);
                }

                telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
                telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
                telemetry.addData("Hood pos: ", hoodPos);
                telemetry.addLine("---------------------------------");
                telemetry.addData("X", mecanumCommand.getX());
                telemetry.addData("Y", mecanumCommand.getY());
                telemetry.addData("Theta", mecanumCommand.getOdoHeading());
                telemetry.addData("Outtake speed: ", shootSpeed);
                telemetry.update();
            }
        }
    }
