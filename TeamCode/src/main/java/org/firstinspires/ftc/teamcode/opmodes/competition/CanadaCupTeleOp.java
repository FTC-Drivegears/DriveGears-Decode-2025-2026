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
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.coloursensor.ColourSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismTutorial;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

import java.util.Arrays;
import java.util.List;

/**
 * CanadaCup Competition TeleOp — ALL CSV LOGGING DISABLED.
 * Use DiagnosticTeleOp for testing with full logging.
 *
 * Anti-freeze build:
 *   - Both blocking I2C reads (V3 colour sensors, Pinpoint) run on worker
 *     threads inside their subsystems; a wedged sensor can't freeze this loop.
 *   - Heading falls back to robot-centric driving when the odo worker goes stale.
 *   - LoopWatchdog (nested below) cuts drive power if the loop stalls anyway.
 *   - Lynx bulk caching (MANUAL) keeps the Exp-Hub encoder reads cheap.
 */
@TeleOp(name = "CanadaCup", group = "TeleOp")
public class CanadaCupTeleOp extends LinearOpMode {

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

    // TX tolerance by distance — close shots get looser aim requirement
    private static final double TX_TOLERANCE_CLOSE_DEG = 8.0;   // at 0.3 m
    private static final double TX_TOLERANCE_FAR_DEG   = 2.0;   // at 2.5 m
    private static final double DIST_CLOSE_M           = 0.3;
    private static final double DIST_FAR_M             = 2.5;
    private static final double TX_TOLERANCE_DEFAULT   = 3.0;   // fallback if no distance yet

    // RPM tolerance by distance — close shots don't need tight RPM
    private static final double RPM_TOLERANCE_CLOSE = 200.0;  // at 0.3 m
    private static final double RPM_TOLERANCE_FAR   = 80.0;   // at 2.5 m

    private final ElapsedTime sorterTimer       = new ElapsedTime();
    private final ElapsedTime pusherReturnTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        Hardware.resetInstance();
        hw               = Hardware.getInstance(hardwareMap);
        mecanumCommand   = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);
        turret.setMecanumCommand(mecanumCommand);

        // *** DISABLE ALL CSV LOGGING — prevents OOM crash ***
        turret.disableLogging();
        mecanumCommand.disableOdoLogging();

        limelight = hw.limelight;
        limelight.pipelineSwitch(8);
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
        hw.leftLight.setPosition(0.0);
        hw.hood.setPosition(0.50);
        hw.rightLight.setPosition(1.0);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.llmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean autoAimEnabled  = false;
        boolean prevA           = false;
        boolean previousXState  = false;
        boolean previousYState  = false;
        boolean togglePusher    = false;
        boolean isShooterOn     = false;
        boolean prevDpadA       = false;
        boolean pusherReturning = false;
        boolean pusherAtFire    = false;
        boolean rpmLatch        = false;
        boolean prevFullReset   = false;
        int     loopCount       = 0;

        // Preload positions are constant — compute once outside the loop
        final double PRELOAD_FRACTION = 0.70;
        final double preloadR = PusherConsts.PUSHER_DOWN_POSITION_R
                + (PusherConsts.PUSHER_UP_POSITION_R - PusherConsts.PUSHER_DOWN_POSITION_R) * PRELOAD_FRACTION;
        final double preloadL = PusherConsts.PUSHER_DOWN_POSITION_L
                + (PusherConsts.PUSHER_UP_POSITION_L - PusherConsts.PUSHER_DOWN_POSITION_L) * PRELOAD_FRACTION;

        // Turret + flywheel encoders live on the Expansion Hub (relayed over RS-485).
        // MANUAL bulk caching => one bulk read per hub per loop instead of one per call.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : allHubs) m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Drive watchdog — zeros drive if the main loop stalls > 250 ms.
        LoopWatchdog watchdog = new LoopWatchdog(250, hw.lf, hw.rf, hw.lb, hw.rb);
        watchdog.start();

        waitForStart();

        while (opModeIsActive()) {

            // Refresh the per-hub bulk cache once, then pet the watchdog.
            for (LynxModule m : allHubs) m.clearBulkCache();
            watchdog.pet();

            // ===== TEMP DIAGNOSTIC: stall locator. On a freeze, the LAST "PHASE:"
            // line in logcat is the call that hung. Safe to delete now that the
            // blocking reads are off the loop. =====
            RobotLog.ii("PHASE", "odo");
            mecanumCommand.processOdometry();

            double heading = mecanumCommand.getOdoHeading();
            // Robot-centric fallback: zero the heading fed to the field transform
            // whenever odo is dead (NaN or worker gone stale).
            if (Double.isNaN(heading) || mecanumCommand.isHeadingStale()) heading = 0;

            double inputY = -gamepad1.left_stick_y;
            double inputX =  gamepad1.left_stick_x;
            double inputR =  gamepad1.right_stick_x;

            if (gamepad1.x) {
                inputY /= 3;
                inputX /= 3;
                inputR /= 3;
            }

            double fieldX = inputX * Math.cos(-heading) - inputY * Math.sin(-heading);
            double fieldY = inputX * Math.sin(-heading) + inputY * Math.cos(-heading);
            RobotLog.ii("PHASE", "drive");
            theta = mecanumCommand.normalMove(fieldY, fieldX, inputR);
            RobotLog.ii("PHASE", "limelight");
            llResult = limelight.getLatestResult();
            Double tx = (llResult != null && llResult.isValid()) ? llResult.getTx() : null;
            Double ty = (llResult != null && llResult.isValid()) ? llResult.getTy() : null;

            // --- Distance-scaled tolerances ---
            double dist = turret.getDistanceTrack();
            double txTolerance;
            double rpmTolerance;
            if (dist <= 0) {
                txTolerance  = TX_TOLERANCE_DEFAULT;
                rpmTolerance = RPM_TOLERANCE_FAR;
            } else {
                double normalized = Range.clip(
                        (dist - DIST_CLOSE_M) / (DIST_FAR_M - DIST_CLOSE_M), 0.0, 1.0);
                txTolerance  = TX_TOLERANCE_CLOSE_DEG
                        - normalized * (TX_TOLERANCE_CLOSE_DEG - TX_TOLERANCE_FAR_DEG);
                rpmTolerance = RPM_TOLERANCE_CLOSE
                        - normalized * (RPM_TOLERANCE_CLOSE - RPM_TOLERANCE_FAR);
            }
            shooterSubsystem.setRPMTolerance(rpmTolerance);

            // --- Auto aim toggle ---
            boolean curA = gamepad2.start;
            if (curA && !prevA) {
                autoAimEnabled = !autoAimEnabled;
                leftLight.setPosition(autoAimEnabled ? 0.44 : 0.0);
            }
            prevA = curA;

            // --- Turret ---
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

            boolean isManual = (manualPower != 0);

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

            // --- Intake ---
            boolean isIntakeMotorOn  = gamepad1.right_trigger > 0.5;
            boolean isOuttakeMotorOn = gamepad1.left_trigger  > 0.5;
            if (isIntakeMotorOn) isOuttakeMotorOn = false;

            RobotLog.ii("PHASE", "colour");
            colourSubsystem.update(isIntakeMotorOn);

            if (isIntakeMotorOn)       intake.setPower(0.8);
            else if (isOuttakeMotorOn) intake.setPower(-0.8);
            else                       intake.setPower(0);

            // --- Shooter toggle ---
            boolean curX = gamepad2.x;
            if (curX && !previousXState) {
                isShooterOn = !isShooterOn;
                if (isShooterOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    shooterSubsystem.spinup();
                } else {
                    shooterSubsystem.stopShooter();
                    rpmLatch = false;
                    leftLight.setPosition(autoAimEnabled ? 0.44 : 0.0);
                }
            }
            previousXState = curX;

            if (isShooterOn) {
                shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                shooterSubsystem.spinup();
                if (shooterSubsystem.isRPMReached()) rpmLatch = true;
                if (tx != null && Math.abs(tx) < txTolerance && shooterSubsystem.isRPMReached()) {
                    leftLight.setPosition(0.3);
                } else {
                    leftLight.setPosition(autoAimEnabled ? 0.44 : 0.0);
                }
            }

            // --- Pusher ---
            // Servo positions are only written on state transitions, never every loop,
            // to prevent the servo hunting/jiggling while holding the button.
            boolean curY     = gamepad2.y;
            boolean yPressed = curY && !previousYState;

            if (curY) {
                if (yPressed) {
                    // Initial press — go to preload or straight to fire if already ready
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
                    // Already holding Y and conditions just became met — upgrade to fire once
                    pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                    pusherAtFire = true;
                }
                // No else — don't touch servo if nothing changed
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
            previousYState = curY;

            if (pusherReturning && pusherReturnTimer.milliseconds() >= 400) {
                pusherReturning = false;
                sorterSubsystem.removeCurrentBall();
                sorterSubsystem.manualSpinReverse();
            }

            // --- Sorter manual controls ---
            if (gamepad2.dpad_left && sorterTimer.milliseconds() > 500) {
                sorterTimer.reset();
                sorterSubsystem.manualSpin();
            }
            if (gamepad2.dpad_right && sorterTimer.milliseconds() > 500) {
                sorterTimer.reset();
                sorterSubsystem.manualSpinReverse();
            }

            // --- Colour selection ---
            if (gamepad2.left_trigger > 0.5) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.GREEN;
                rightLight.setPosition(0.5);
            }
            if (gamepad2.right_trigger > 0.5) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.PURPLE;
                rightLight.setPosition(0.722);
            }
            if (gamepad2.back) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.ANY;
                rightLight.setPosition(1.0);
            }

            // --- Quickfire ---
            if (gamepad2.a && !prevDpadA) sorterSubsystem.startQuickfire();
            RobotLog.ii("PHASE", "quickfire");
            if (sorterSubsystem.isActive()) sorterSubsystem.quickfireState();
            prevDpadA = gamepad2.a;
            if (gamepad2.b) sorterSubsystem.stopQuickfire();

            // --- Odometry reset ---
            // start = full reset: zero XY AND recalibrate IMU heading.
            // Edge-triggered so holding the button doesn't restart the IMU
            // calibration every loop. resetPosAndIMU() samples the gyro for
            // ~0.25s, so press this only while the robot is stationary.
            boolean fullReset = gamepad1.start;
            if (fullReset && !prevFullReset) {
                mecanumCommand.resetPinPointOdometry();
            }
            prevFullReset = fullReset;

            // --- Telemetry (throttled) ---
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
                telemetry.addData("Odo stale",    mecanumCommand.isHeadingStale());
                telemetry.addData("Colour stale", colourSubsystem.isStale());
                telemetry.addData("Watchdog",     watchdog.isTripped() ? "TRIPPED" : "ok");
                telemetry.addLine("---");
                telemetry.addData("Balls",        Arrays.toString(sorterSubsystem.getSorterList()));
                telemetry.addData("Sorter pos",   sorterSubsystem.getSorterPos());
                telemetry.addData("Colour",       sorterSubsystem.selectedColour);
                telemetry.addData("Ball present", colourSubsystem.isBallPresent());
                telemetry.addData("Sensors",      colourSubsystem.isSensor1Ok() + " / " + colourSubsystem.isSensor2Ok());
                telemetry.update();
            }
            RobotLog.ii("PHASE", "end");
        }

        // Loop exited normally (STOP pressed) — shut the worker threads down.
        watchdog.stop();
        mecanumCommand.stopOdometry();
        colourSubsystem.stop();
    }

    /**
     * Cuts drive power if the main loop stops petting it. Backstop only — does not
     * recover a hung read. Can only zero motors on a live hub, which is why the
     * drivetrain sits alone on the Control Hub.
     */
    private static class LoopWatchdog {
        private final DcMotor[] drive;
        private final long stallMs;
        private volatile long    lastPet = System.currentTimeMillis();
        private volatile boolean running = false;
        private volatile boolean tripped = false;
        private Thread thread;

        LoopWatchdog(long stallMs, DcMotor... drive) {
            this.stallMs = stallMs;
            this.drive   = drive;
        }

        void start() {
            running = true;
            lastPet = System.currentTimeMillis();
            thread = new Thread(() -> {
                while (running) {
                    if (System.currentTimeMillis() - lastPet > stallMs) {
                        tripped = true;
                        for (DcMotor m : drive) {
                            try { m.setPower(0); } catch (Exception ignored) {}
                        }
                    }
                    try { Thread.sleep(20); } catch (InterruptedException e) { break; }
                }
            }, "LoopWatchdog");
            thread.setDaemon(true);
            thread.start();
        }

        void pet()          { lastPet = System.currentTimeMillis(); tripped = false; }
        boolean isTripped() { return tripped; }
        void stop()         { running = false; if (thread != null) thread.interrupt(); }
    }
}