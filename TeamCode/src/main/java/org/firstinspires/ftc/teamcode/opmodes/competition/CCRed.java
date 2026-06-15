package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.coloursensor.ColourSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismTutorial;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;
import org.firstinspires.ftc.teamcode.util.FieldOrientedOffset;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

import java.util.Arrays;

@TeleOp(name = "CanadaCupRed", group = "TeleOp")
public class CCRed extends LinearOpMode {

    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private ShooterSubsystem shooterSubsystem;
    private SorterSubsystem sorterSubsystem;
    private TurretMechanismTutorial turret;
    private ColourSensorSubsystem colourSubsystem;

    private Limelight3A limelight;
    private LLResult llResult;

    private DcMotor intake;
    private Servo pusher_R, pusher_L, leftLight, rightLight;

    private double theta;

    private static final double TICKS_PER_DEGREE     = 1.8;
    private static final double HARD_LIMIT_CW_DEG    =  150.0;
    private static final double HARD_LIMIT_CCW_DEG   = -190.0;
    private static final double MANUAL_LIMIT_CW_DEG  =  135.0;
    private static final double MANUAL_LIMIT_CCW_DEG = -180.0;

    private static final double TX_TOLERANCE_CLOSE_DEG = 8.0;
    private static final double TX_TOLERANCE_FAR_DEG   = 2.0;
    private static final double DIST_CLOSE_M           = 0.3;
    private static final double DIST_FAR_M             = 2.5;
    private static final double TX_TOLERANCE_DEFAULT   = 3.0;

    private static final double RPM_TOLERANCE_CLOSE = 200.0;
    private static final double RPM_TOLERANCE_FAR   = 80.0;

    private static final double LIGHT_OFF    = 0.0;
    private static final double LIGHT_READY  = 0.3;
    private static final double LIGHT_AUTO   = 0.44;

    private static final double RIGHT_NONE   = 1.0;
    private static final double RIGHT_GREEN  = 0.5;
    private static final double RIGHT_PURPLE = 0.722;

    private final ElapsedTime sorterTimer       = new ElapsedTime();
    private final ElapsedTime pusherReturnTimer = new ElapsedTime();
    private final ElapsedTime rightLightTimer   = new ElapsedTime();
    private final ElapsedTime rescanTimer       = new ElapsedTime();

    private enum RescanState {
        IDLE,
        WAIT_SAMPLE
    }

    private RescanState rescanState = RescanState.IDLE;
    private int rescanSamplesDone = 0;

    private void updateLeftTurretLight(boolean autoAimEnabled, boolean turretReady) {
        if (turretReady) {
            leftLight.setPosition(LIGHT_READY);
        } else if (autoAimEnabled) {
            leftLight.setPosition(LIGHT_AUTO);
        } else {
            leftLight.setPosition(LIGHT_OFF);
        }
    }

    private void updateRightBallLight(
            Artifact.BallColour currentBallColour,
            SorterSubsystem.SelectedColour selectedColour
    ) {
        double currentColourPosition = ballColourToRightLight(currentBallColour);

        if (selectedColour == SorterSubsystem.SelectedColour.ANY) {
            rightLight.setPosition(currentColourPosition);
            return;
        }

        double selectedColourPosition = selectedColourToRightLight(selectedColour);

        boolean showSelected = ((int) (rightLightTimer.milliseconds() / 250)) % 2 == 0;
        rightLight.setPosition(showSelected ? selectedColourPosition : 1.0);
    }

    private double ballColourToRightLight(Artifact.BallColour colour) {
        if (colour == Artifact.BallColour.GREEN) return RIGHT_GREEN;
        if (colour == Artifact.BallColour.PURPLE) return RIGHT_PURPLE;
        return RIGHT_NONE;
    }

    private double selectedColourToRightLight(SorterSubsystem.SelectedColour colour) {
        if (colour == SorterSubsystem.SelectedColour.GREEN) return RIGHT_GREEN;
        if (colour == SorterSubsystem.SelectedColour.PURPLE) return RIGHT_PURPLE;
        return RIGHT_NONE;
    }

    private void startSorterRescan() {
        sorterSubsystem.stopQuickfire();
        sorterSubsystem.clearSorterList();

        rescanSamplesDone = 0;
        rescanState = RescanState.WAIT_SAMPLE;
        rescanTimer.reset();
    }

    private boolean isRescanningSorter() {
        return rescanState != RescanState.IDLE;
    }

    private void updateSorterRescan() {
        if (rescanState == RescanState.IDLE) return;
        if (rescanTimer.milliseconds() < sorterSubsystem.getCurrentSettleTimeMs()) return;

        Artifact.BallColour detectedColour = colourSubsystem.sampleCurrentBallColour();
        sorterSubsystem.setCurrentSlotColour(detectedColour);

        rescanSamplesDone++;

        if (rescanSamplesDone >= SorterSubsystem.MAX_NUM_BALLS) {
            sorterSubsystem.manualSpin();
            rescanState = RescanState.IDLE;
            return;
        }

        sorterSubsystem.manualSpin();
        rescanTimer.reset();
    }

    @Override
    public void runOpMode() {

        Hardware.resetInstance();
        hw               = Hardware.getInstance(hardwareMap);
        mecanumCommand   = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);
        turret.setMecanumCommand(mecanumCommand);

        turret.disableLogging();
        mecanumCommand.disableOdoLogging();

        limelight = hw.limelight;
        limelight.pipelineSwitch(7);
        limelight.start();

        sorterSubsystem = new SorterSubsystem(hw, shooterSubsystem, this, telemetry, "pgg");
        colourSubsystem = new ColourSensorSubsystem(hardwareMap, hw, sorterSubsystem);

        intake     = hw.intake;
        pusher_R   = hw.pusher_R;
        pusher_L   = hw.pusher_L;
        leftLight  = hw.leftLight;
        rightLight = hw.rightLight;

        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
        hw.sorter.setPosition(sorterSubsystem.getFirstSorterPos());

        leftLight.setPosition(LIGHT_OFF);
        rightLight.setPosition(RIGHT_NONE);
        hw.hood.setPosition(0.50);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.llmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean autoAimEnabled   = false;

        boolean prevA            = false;
        boolean previousXState   = false;
        boolean previousYState   = false;
        boolean togglePusher     = false;
        boolean isShooterOn      = false;
        boolean prevQuickfireA   = false;
        boolean pusherReturning  = false;
        boolean pusherAtFire     = false;
        boolean rpmLatch         = false;
        boolean prevFullReset    = false;
        boolean prevRescanButton = false;

        int loopCount = 0;

        final double PRELOAD_FRACTION = 0.70;
        final double preloadR = PusherConsts.PUSHER_DOWN_POSITION_R
                + (PusherConsts.PUSHER_UP_POSITION_R - PusherConsts.PUSHER_DOWN_POSITION_R) * PRELOAD_FRACTION;
        final double preloadL = PusherConsts.PUSHER_DOWN_POSITION_L
                + (PusherConsts.PUSHER_UP_POSITION_L - PusherConsts.PUSHER_DOWN_POSITION_L) * PRELOAD_FRACTION;

        waitForStart();

        // ---------------------------------------------------------------
        // try/finally: guarantees the odometry worker thread is stopped on
        // exit (clean stop, exception, or SDK kill).  A leaked odo thread
        // contends on the I2C bus in the next run and hangs the colour
        // sensor — this single block is the fix.
        // ---------------------------------------------------------------
        try {
            while (opModeIsActive()) {

                RobotLog.ii("PHASE", "odo");
                mecanumCommand.processOdometry();

                double rawHeading = mecanumCommand.getOdoHeading();
                if (Double.isNaN(rawHeading)) rawHeading = 0;

                double heading = rawHeading + FieldOrientedOffset.headingOffsetRad;

                double inputY = -gamepad1.left_stick_y;
                double inputX =  gamepad1.left_stick_x;
                double inputR =  gamepad1.right_stick_x;

                if (gamepad1.left_bumper) {
                    inputY /= 2;
                    inputX /= 2;
                    inputR /= 2;
                }

                if (gamepad1.right_bumper) {
                    inputY /= 2;
                    inputX /= 2;
                    inputR /= 2;
                }

                double driveX = inputX * Math.cos(-heading) - inputY * Math.sin(-heading);
                double driveY = inputX * Math.sin(-heading) + inputY * Math.cos(-heading);

                RobotLog.ii("PHASE", "drive");
                theta = mecanumCommand.normalMove(driveY, driveX, inputR);

                RobotLog.ii("PHASE", "limelight");
                llResult = limelight.getLatestResult();
                Double tx = (llResult != null && llResult.isValid()) ? llResult.getTx() : null;
                Double ty = (llResult != null && llResult.isValid()) ? llResult.getTy() : null;

                double dist = turret.getDistanceTrack();
                double txTolerance;
                double rpmTolerance;

                if (dist <= 0) {
                    txTolerance  = TX_TOLERANCE_DEFAULT;
                    rpmTolerance = RPM_TOLERANCE_FAR;
                } else {
                    double normalized = Range.clip(
                            (dist - DIST_CLOSE_M) / (DIST_FAR_M - DIST_CLOSE_M),
                            0.0,
                            1.0
                    );

                    txTolerance = TX_TOLERANCE_CLOSE_DEG
                            - normalized * (TX_TOLERANCE_CLOSE_DEG - TX_TOLERANCE_FAR_DEG);

                    rpmTolerance = RPM_TOLERANCE_CLOSE
                            - normalized * (RPM_TOLERANCE_CLOSE - RPM_TOLERANCE_FAR);
                }

                shooterSubsystem.setRPMTolerance(rpmTolerance);

                boolean curA = gamepad2.start;
                if (curA && !prevA) {
                    autoAimEnabled = !autoAimEnabled;
                }
                prevA = curA;

                double turretDeg = hw.llmotor.getCurrentPosition() / TICKS_PER_DEGREE;

                double manualPower = 0;
                if (gamepad2.left_bumper) {
                    if (turretDeg >= MANUAL_LIMIT_CW_DEG) manualPower = 0;
                    else {
                        double scale = Math.min((MANUAL_LIMIT_CW_DEG - turretDeg) / 30.0, 1.0);
                        manualPower = 0.15 + 0.25 * scale;
                    }
                } else if (gamepad2.right_bumper) {
                    if (turretDeg <= MANUAL_LIMIT_CCW_DEG) manualPower = 0;
                    else {
                        double scale = Math.min((turretDeg - MANUAL_LIMIT_CCW_DEG) / 30.0, 1.0);
                        manualPower = -(0.15 + 0.25 * scale);
                    }
                }

                boolean isManual = manualPower != 0;

                if (turretDeg > HARD_LIMIT_CW_DEG) {
                    hw.llmotor.setPower(-0.3);
                } else if (turretDeg < HARD_LIMIT_CCW_DEG) {
                    hw.llmotor.setPower(0.3);
                } else if (isManual) {
                    hw.llmotor.setPower(manualPower);
                } else if (autoAimEnabled) {
                    turret.update(tx, ty);
                } else {
                    hw.llmotor.setPower(0);
                }

                loopCount++;

                boolean isIntakeMotorOn  = gamepad1.right_trigger > 0.5;
                boolean isOuttakeMotorOn = gamepad1.left_trigger  > 0.5;
                if (isIntakeMotorOn) isOuttakeMotorOn = false;

                boolean rescanButton = gamepad1.a;
                if (rescanButton && !prevRescanButton && !isRescanningSorter()) {
                    startSorterRescan();
                }
                prevRescanButton = rescanButton;

                RobotLog.ii("PHASE", "colour");

                if (isRescanningSorter()) {
                    updateSorterRescan();
                } else {
                    colourSubsystem.update(isIntakeMotorOn);
                }

                if (isIntakeMotorOn)       intake.setPower(0.8);
                else if (isOuttakeMotorOn) intake.setPower(-0.8);
                else                       intake.setPower(0);

                boolean curX = gamepad2.x;
                if (curX && !previousXState) {
                    isShooterOn = !isShooterOn;
                    if (isShooterOn) {
                        shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                        shooterSubsystem.spinup();
                    } else {
                        shooterSubsystem.stopShooter();
                        rpmLatch = false;
                    }
                }
                previousXState = curX;

                boolean turretReady = false;

                if (isShooterOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    shooterSubsystem.spinup();

                    if (shooterSubsystem.isRPMReached()) rpmLatch = true;

                    turretReady = tx != null
                            && Math.abs(tx) < txTolerance
                            && shooterSubsystem.isRPMReached();
                }

                updateLeftTurretLight(autoAimEnabled, turretReady);

                boolean curY     = gamepad2.y;
                boolean yPressed = curY && !previousYState;

                if (!isRescanningSorter()) {
                    if (curY) {
                        if (yPressed) {
                            if (rpmLatch && tx != null && Math.abs(tx) < txTolerance) {
                                pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                                pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                                pusherAtFire = true;
                            } else {
                                pusher_R.setPosition(preloadR);
                                pusher_L.setPosition(preloadL);
                            }
                            togglePusher = true;
                        } else if (!pusherAtFire && rpmLatch && tx != null && Math.abs(tx) < txTolerance) {
                            pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                            pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                            pusherAtFire = true;
                        }
                    } else {
                        if (togglePusher) {
                            pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                            pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                            togglePusher    = false;
                            pusherAtFire    = false;
                            rpmLatch        = false;
                            pusherReturning = true;
                            pusherReturnTimer.reset();
                        }
                    }
                }

                previousYState = curY;

                if (pusherReturning && pusherReturnTimer.milliseconds() >= 400) {
                    pusherReturning = false;
                    sorterSubsystem.removeCurrentBall();
                    sorterSubsystem.manualSpinReverse();
                }

                if (!isRescanningSorter()) {
                    if (gamepad2.dpad_left && sorterTimer.milliseconds() > 500) {
                        sorterTimer.reset();
                        sorterSubsystem.manualSpin();
                    }

                    if (gamepad2.dpad_right && sorterTimer.milliseconds() > 500) {
                        sorterTimer.reset();
                        sorterSubsystem.manualSpinReverse();
                    }
                }

                if (gamepad2.left_trigger > 0.5) {
                    sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.GREEN;
                }

                if (gamepad2.right_trigger > 0.5) {
                    sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.PURPLE;
                }

                if (gamepad2.back) {
                    sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.ANY;
                }

                updateRightBallLight(
                        colourSubsystem.getCurrentBallColour(),
                        sorterSubsystem.selectedColour
                );

                if (!isRescanningSorter()) {
                    if (gamepad2.a && !prevQuickfireA) {
                        sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.ANY;
                        sorterSubsystem.startQuickfire();
                    }

                    RobotLog.ii("PHASE", "quickfire");

                    if (sorterSubsystem.isActive()) {
                        Artifact.BallColour quickfireSensedColour = Artifact.BallColour.NONE;
                        boolean quickfireBallPresent = false;

                        if (!sorterSubsystem.isSorterSettling()) {
                            quickfireSensedColour = colourSubsystem.sampleCurrentBallColour();
                            quickfireBallPresent = colourSubsystem.isBallPresent();
                        }

                        sorterSubsystem.quickfireState(quickfireSensedColour, quickfireBallPresent);
                    }

                    if (gamepad2.b) {
                        sorterSubsystem.stopQuickfire();
                    }
                }

                prevQuickfireA = gamepad2.a;

                boolean fullReset = gamepad1.start;
                if (fullReset && !prevFullReset) {
                    mecanumCommand.resetPinPointOdometry();
                }
                prevFullReset = fullReset;

                RobotLog.ii("PHASE", "telemetry");
                if (loopCount % 4 == 0) {
                    telemetry.addData("Turret",       String.format("%.1f°", turretDeg));
                    telemetry.addData("Has Target",   turret.hasTarget());
                    telemetry.addData("tx",           tx);
                    telemetry.addData("tx tolerance", String.format("%.1f°", txTolerance));
                    telemetry.addData("Distance",     String.format("%.2f m", dist));
                    telemetry.addData("Shooter RPM",  turret.getShootRPM());
                    telemetry.addData("Auto Aim",     autoAimEnabled);
                    telemetry.addData("Shooter on",   isShooterOn);
                    telemetry.addData("RPM Latched",  rpmLatch);
                    telemetry.addLine("---");
                    telemetry.addData("Drive Mode",   "FIELD");
                    telemetry.addData("Heading Raw",  rawHeading);
                    telemetry.addData("Heading Deg",  Math.toDegrees(heading));
                    telemetry.addLine("---");
                    telemetry.addData("Balls",        Arrays.toString(sorterSubsystem.getSorterList()));
                    telemetry.addData("Ball Count",   sorterSubsystem.getArtifactCount());
                    telemetry.addData("Sorter pos",   sorterSubsystem.getSorterPos());
                    telemetry.addData("Sorter settling", sorterSubsystem.isSorterSettling());
                    telemetry.addData("Rescanning",   isRescanningSorter());
                    telemetry.addData("Quickfire",    sorterSubsystem.quickfireState);
                    telemetry.addData("Selected",     sorterSubsystem.selectedColour);
                    telemetry.addData("Current Ball", colourSubsystem.getCurrentBallColour());
                    telemetry.addData("Ball present", colourSubsystem.isBallPresent());
                    telemetry.addData("Sensors",      colourSubsystem.isSensor1Ok() + " / " + colourSubsystem.isSensor2Ok());

                    telemetry.addData("C1 RGBA", "%.3f %.3f %.3f %.3f",
                            colourSubsystem.getRed(),
                            colourSubsystem.getGreen(),
                            colourSubsystem.getBlue(),
                            colourSubsystem.getAlpha());

                    telemetry.addData("C2 RGBA", "%.3f %.3f %.3f %.3f",
                            colourSubsystem.getRed2(),
                            colourSubsystem.getGreen2(),
                            colourSubsystem.getBlue2(),
                            colourSubsystem.getAlpha2());

                    telemetry.update();
                }

                RobotLog.ii("PHASE", "end");
            }
        } finally {
            // ALWAYS runs — clean stop, exception, or SDK kill.
            // Stops the odometry worker thread so it cannot leak into the
            // next run and contend on the I2C bus with the colour sensors.
            try { mecanumCommand.stopOdometry(); } catch (Exception ignored) {}
        }
    }
}