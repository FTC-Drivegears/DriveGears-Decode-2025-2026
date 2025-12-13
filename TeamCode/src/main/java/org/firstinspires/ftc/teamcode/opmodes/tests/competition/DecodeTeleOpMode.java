package org.firstinspires.ftc.teamcode.opmodes.tests.competition;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class DecodeTeleOpMode extends LinearOpMode {
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
    private Servo gate;
    private DcMotorEx llmotor;
    private Limelight3A limelight;

    private SorterSubsystem sorterSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private final ElapsedTime sorterTimer = new ElapsedTime();

    private final ElapsedTime outtakeTimer = new ElapsedTime();

    private final ElapsedTime pusherTimer = new ElapsedTime();

    private final ElapsedTime colorSensingTimer = new ElapsedTime();

    //Shooter Presets
    private final double FAR_HOOD = 0.4;
    private final int FAR_SHOOT_SPEED = 3700;
    private final double MID_HOOD = 0.6;
    private final int MID_SHOOT_SPEED = 3050;
    private final double CLOSE_HOOD = 0.846;
    private final int CLOSE_SHOOT_SPEED = 2500;

    private static final double GATE_UP = 1.0;
    private static final double GATE_DOWN = 0.65;

    private LogitechVisionSubsystem vision;

    private enum DRIVETYPE {
        ROBOTORIENTED, FIELDORIENTED
    }


//    enum OUTTAKE {
//
//        PUSHUP1,
//        PUSHDOWN1,
//        SORT1,
//        PUSHUP2,
//        PUSHDOWN2,
//        SORT2,
//        PUSHUP3,
//        PUSHDOWN3,
//        IDLE
//    }

//    OUTTAKE outtakeState = OUTTAKE.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousXState = false;
        boolean previousYState = false;

        boolean currentXState;
        boolean currentYState;

        boolean intaking = false;
        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean rightTriggerPressed = false;
        boolean leftTriggerPressed = false;

        double hoodPos = 0.846;
        double shootSpeed = 4000;

        boolean autoAimState = false;
        boolean previousAimButton = false;

        boolean llDetection = true;

        DRIVETYPE drivetype = DRIVETYPE.ROBOTORIENTED;

        hw = Hardware.getInstance(hardwareMap);

        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        vision = new LogitechVisionSubsystem(hw, "RED");
        pusher = hw.pusher;
        light = hw.light;
        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.light.setPosition(0.0);
        hw.sorter.setPosition(0.085);
        hw.gate.setPosition(GATE_DOWN);
        hw.hood.setPosition(hoodPos);

        intake = hw.intake;
        shooter = hw.shooter;
        hood = hw.hood;
        gate = hw.gate;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llmotor = hardwareMap.get(DcMotorEx.class, "llmotor");

        limelight.pipelineSwitch(7);
        limelight.start();

        telemetry.addLine("waiting for start");
        telemetry.update();

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        if (sorterSubsystem == null) {
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "");
        }

        while (opModeInInit()) {
            if (gamepad1.a) {
                drivetype = DRIVETYPE.ROBOTORIENTED;
            }

            if (gamepad1.y) {
                drivetype = DRIVETYPE.FIELDORIENTED;
            }

            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            Double headingError = vision.getTargetYaw();
            boolean targetFound = (headingError != null);

            double turn = 0;

//            if (gamepad2.left_bumper && !previousAimButton) {
//                autoAimState = !autoAimState;   // toggle on *edge* of button press
//            }
//            previousAimButton = gamepad2.left_bumper;
//
//            if (autoAimState) {
//                if (LogitechVisionSubsystem.targetApril(telemetry) > 5) {
//                    mecanumCommand.pivot(0.2);
//                } else if (LogitechVisionSubsystem.targetApril(telemetry) < -5) {
//                    mecanumCommand.pivot(-0.2);
//                } else {
//                    mecanumCommand.pivot(0);
//                }
//            }

            mecanumCommand.processOdometry();
//            sorterSubsystem.transfer();

//            if (drivetype == DRIVETYPE.FIELDORIENTED) {
//                theta = mecanumCommand.fieldOrientedMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (drivetype == DRIVETYPE.ROBOTORIENTED) {
                theta = mecanumCommand.robotOrientedMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            //intake
            if (gamepad1.right_trigger > 0) {
                if (!rightTriggerPressed) {
                    rightTriggerPressed = true;
                    isIntakeMotorOn = !isIntakeMotorOn;
                    if (isIntakeMotorOn) {
                        intake.setPower(0.8);
                        gate.setPosition(GATE_UP);
//                        intaking = true;
                    } else {
                        intake.setPower(0);
                        gate.setPosition(GATE_DOWN);
//                        intaking = false;
                    }
                }
            } else
                rightTriggerPressed = false;

            if (gamepad1.left_trigger > 0) {
                if (!leftTriggerPressed) {
                    leftTriggerPressed = true;
                    isIntakeMotorOn = !isIntakeMotorOn;
                    if (isIntakeMotorOn) {
                        intake.setPower(-0.8);
                        gate.setPosition(GATE_UP);
                    } else {
                        intake.setPower(0);
                        gate.setPosition(GATE_DOWN);
                    }
                }
            } else {
                leftTriggerPressed = false;
            }

            if (gamepad1.b && sorterTimer.milliseconds() > 500) {
                sorterTimer.reset();
                sorterSubsystem.manualSpin();
            }


//            boolean right = gamepad1.dpad_right;
//            boolean left = gamepad1.dpad_left;
//            if (right || left) { // right to spin sorter to green for outtake, left to spin sorter to purple for outtake
//                if (outtakeTimer.milliseconds() > 500) {
//                    char curColor = 'g';
//                    if (left) {
//                        curColor = 'p';
//                    }
//                    sorterSubsystem.outtakeBall(curColor);
//                    outtakeTimer.reset();
//                }
//            }
            if (intaking && colorSensingTimer.milliseconds() > 500) {
                sorterSubsystem.detectColor();
                if (sorterSubsystem.getIsBall()) {
                    sorterSubsystem.turnToIntake('P');
                    colorSensingTimer.reset();
                }
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

//            if (gamepad1.a && sorterSubsystem.state == SorterSubsystem.TransferState.FIRST) {
//                sorterSubsystem.state = SorterSubsystem.TransferState.PUSH_UP;
//            }

            // CLOSE
            if (gamepad2.x) {
                hood.setPosition(CLOSE_HOOD);
                shootSpeed = CLOSE_SHOOT_SPEED;

            }
            //MID
            if (gamepad2.y) {
                hood.setPosition(MID_HOOD);
                shootSpeed = MID_SHOOT_SPEED;
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
                    light.setPosition(0.333);
                } else {
                    light.setPosition(0.0);
                }

            } else {
                shooterSubsystem.stopShooter();
                light.setPosition(0.0);
            }

            //TURRET

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
                double tx = result.getTx();
                if (tx > 3) {
                    llmotor.setPower(-0.5);
                } else if (tx < -3) {
                    llmotor.setPower(0.5);
                } else {
                    llmotor.setPower(0);
                }
                telemetry.addData("tx", tx);
                telemetry.update();

            } else {
                if (gamepad2.dpad_right) {
                    llmotor.setPower(-0.5);
                } else if (gamepad2.dpad_left) {
                    llmotor.setPower(0.5);
                } else {
                    llmotor.setPower(0);
                }
            }

//            if (gamepad2.a) {
//                llDetection = false;
//                if (gamepad2.dpad_right) {
//                    llmotor.setPower(-0.5);
//                } else if (gamepad2.dpad_left) {
//                    llmotor.setPower(0.5);
//                } else {
//                    llmotor.setPower(0);
//                }
//            } else {
//                llDetection = true;
//            }
//
//            if (llDetection) {
//                LLStatus status = limelight.getStatus();
//                telemetry.addData("LL Name", status.getName());
//                telemetry.addData("CPU", "%.1f %%", status.getCpu());
//                telemetry.addData("FPS", "%d", (int) status.getFps());
//                telemetry.addData("Pipeline", "%d (%s)",
//                        status.getPipelineIndex(),
//                        status.getPipelineType()
//                );

//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    double tx = result.getTx();
//                    if (tx > 2.5) {
//                        llmotor.setPower(-0.2);
//                    } else if (tx < -2.5) {
//                        llmotor.setPower(0.2);
//                    } else {
//                        llmotor.setPower(0);
//                    }
//
//                    telemetry.addData("tx", tx);
//                    telemetry.update();
//
//                } else {
//                    if (gamepad2.dpad_right) {
//                        llmotor.setPower(-0.2);
//                    } else if (gamepad2.dpad_left) {
//                        llmotor.setPower(0.2);
//                    } else {
//                        llmotor.setPower(0);
//                    }
                }


            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.addData("colour?: ", sorterSubsystem.getIsBall());
            telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
            telemetry.addData("Hood pos: ", hoodPos);
            telemetry.addLine("---------------------------------");
            telemetry.addData("X", mecanumCommand.getX());
            telemetry.addData("Y", mecanumCommand.getY());
            telemetry.addData("Theta", mecanumCommand.getOdoHeading());
            telemetry.addData("Outtake speed: ", shootSpeed);
            telemetry.addData("Sorter: ", sorterSubsystem.getCurSorterPositionIndex());
            telemetry.update();

        }
    }



//        public void quickfire () {
//            switch (outtakeState) {
//                case IDLE:
//                    break;
//
//                case PUSHUP1:
//                    hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
//                    sorterSubsystem.setIsPusherUp(true);
//                    pusherTimer.reset();
//                    outtakeState = OUTTAKE.PUSHDOWN1;
//                    break;
//
//                case PUSHDOWN1:
//                    if (pusherTimer.milliseconds() >= 1000) {
//                        hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
//                        sorterSubsystem.setIsPusherUp(false);
//                        pusherTimer.reset();
//                        outtakeState = OUTTAKE.SORT1;
//                    }
//                    break;
//
//                case SORT1:
//                    sorterSubsystem.outtakeToNextPos();
//                    outtakeState = OUTTAKE.PUSHUP2;
//                    sorterTimer.reset();
//                    break;
//
//                case PUSHUP2:
//                    if (sorterTimer.milliseconds() >= 1000) {
//                        hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
//                        sorterSubsystem.setIsPusherUp(true);
//                        pusherTimer.reset();
//                        outtakeState = (sorterSubsystem.getCurSorterPositionIndex() > 0) ? OUTTAKE.SORT2 : OUTTAKE.IDLE;
//                    }
//                    break;
//
//                case PUSHDOWN2:
//                    if (pusherTimer.milliseconds() >= 1000) {
//                        sorterSubsystem.setIsPusherUp(false);
//                        hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
//                        outtakeState = OUTTAKE.SORT2;
//                    }
//
//                    break;
//
//                case SORT2:
//                    sorterSubsystem.turnToNextPos();
//                    outtakeState = OUTTAKE.PUSHUP3;
//                    break;
//
//                case PUSHUP3:
//                    if (sorterTimer.milliseconds() >= 1000) {
//                        hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
//                        sorterSubsystem.setIsPusherUp(true);
//                        pusherTimer.reset();
//                        outtakeState = OUTTAKE.PUSHDOWN3;
//                    }
//                    break;
//
//                case PUSHDOWN3:
//                    if (pusherTimer.milliseconds() >= 1000) {
//                        sorterSubsystem.setIsPusherUp(false);
//                        hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
//                        outtakeState = OUTTAKE.IDLE;
//                    }
//                    break;
//            }
//        }
