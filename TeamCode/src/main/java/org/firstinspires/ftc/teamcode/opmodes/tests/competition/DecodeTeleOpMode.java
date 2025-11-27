package org.firstinspires.ftc.teamcode.opmodes.tests.competition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
public class DecodeTeleOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private Hardware hw;
    private double theta;
    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private Servo hood;
    private Servo light;
    private SorterSubsystem sorterSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private long lastIntakeTime;
    private long lastFireTime;
    private long lastOuttakeTime;

    boolean isManualPushOn;

    private final ElapsedTime sorterTimer = new ElapsedTime();

    private final ElapsedTime pusherTimer = new ElapsedTime();

    private final double FAR_HOOD = 0.5;
    private final int FAR_SHOOT_SPEED = 3900;
    private final double MID_HOOD = 0.6;
    private final int MID_SHOOT_SPEED = 3000;
    private final double CLOSE_HOOD = 0.846;
    private final int CLOSE_SHOOT_SPEED = 2500;


    double sorterPosition = 0.0;

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
        double sorterPosition = 0.0;
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
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "");
        }

        while (opModeInInit()) {
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

            curRightTrigger = gamepad1.right_trigger > 0;
            if (curRightTrigger && !prevRightTrigger) {
                isIntakeMotorOn = !isIntakeMotorOn;

                if (isIntakeMotorOn) {
                    intake.setPower(0.8);
                } else {
                    intake.setPower(0);
                }
            }

            curLeftTrigger = gamepad1.left_trigger > 0;
            if (curLeftTrigger && !prevLeftTrigger) {
                isOuttakeMotorOn = !isOuttakeMotorOn;

                if (isOuttakeMotorOn) {
                    intake.setPower(-0.8);
                } else {
                    intake.setPower(0);
                }
            }
            prevRightTrigger = curRightTrigger;


            if (gamepad2.b && sorterTimer.milliseconds() > 1000) {
                sorterPosition = (sorterPosition + 1) % 3;
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
            if (currentYState && !previousYState) {
                togglePusher = !togglePusher;

                if (togglePusher) {
                    isManualPushOn = true;
                    pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
                    sorterSubsystem.setIsPusherUp(true);
                } else {
                    pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
                    sorterSubsystem.setIsPusherUp(false);
                }
            }
            previousYState = currentYState;

            // Pusher
            if (togglePusher && pusherTimer.milliseconds() >= 500) {
                hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
                togglePusher = false;
            }

            currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
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
            if (gamepad2.y) {
                hood.setPosition(FAR_HOOD);
                shootSpeed = FAR_SHOOT_SPEED;

                curRB = gamepad1.right_bumper;
                if (curRB) {
                    if (hoodPos <= 0.359) {
                        hoodPos = 0.359;
                    } else {
                        hoodPos -= 0.001;
                    }
                    hood.setPosition(hoodPos);
                }

                curLB = gamepad1.left_bumper;
                if (curLB) {
                    if (hoodPos >= 0.846) {
                        hoodPos = 0.846;
                    } else {
                        hoodPos += 0.001;
                    }
                    hood.setPosition(hoodPos);
                }

                if (gamepad1.dpad_up) {
                    if (shootSpeed >= 6000.0) {
                        shootSpeed = 6000.0;
                    } else {
//                    shootSpeed += 0.0001;
                        shootSpeed += 150.0;
                        sleep(500);
                    }
                }

                if (gamepad1.dpad_down) {
                    if (shootSpeed <= 0.0) {
                        shootSpeed = 0.0;
                    } else {
//                    shootSpeed -= 0.0001;
                        shootSpeed -= 150.0;
                        sleep(500);
//0.8 default shooter speed
                    }
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
}