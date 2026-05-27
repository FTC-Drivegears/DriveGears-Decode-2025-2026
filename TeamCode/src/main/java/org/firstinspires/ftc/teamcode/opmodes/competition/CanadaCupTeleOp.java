package org.firstinspires.ftc.teamcode.opmodes.competition;

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
import org.firstinspires.ftc.teamcode.util.PusherConsts;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

@TeleOp(name = "CanadaCup", group = "TeleOp")
public class CanadaCupTeleOp extends LinearOpMode {

    // ---------------- SUBSYSTEMS ----------------
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private ShooterSubsystem shooterSubsystem;
    private SorterSubsystem sorterSubsystem;
    private TurretMechanismTutorial turret;
    private ColourSensorSubsystem colourSubsystem;

    // ---------------- LIMELIGHT ----------------
    private Limelight3A limelight;
    private LLResult llResult;

    // ---------------- HARDWARE ----------------
    private DcMotor intake;
    private Servo pusher_R, pusher_L, gate, leftLight, rightLight;

    private double theta;

    // ---------------- TURRET SOFT LIMITS ----------------
    private static final int TURRET_MIN_TICKS = -240;
    private static final int TURRET_MAX_TICKS =  240;

    // ---------------- TIMERS ----------------
    private final ElapsedTime sorterTimer       = new ElapsedTime();
    private final ElapsedTime pusherTimer       = new ElapsedTime();
    private final ElapsedTime pusherReturnTimer  = new ElapsedTime();
    private final ElapsedTime totalTimer        = new ElapsedTime();
    private final ElapsedTime loopDtTimer       = new ElapsedTime();

    // ---------------- PUSHER LOG ----------------
    private BufferedWriter      pusherLog        = null;
    private boolean             pusherLogEnabled = false;
    private final StringBuilder pusherLogLine    = new StringBuilder(256);

    @Override
    public void runOpMode() {

        // ---------------- INIT ----------------
        Hardware.resetInstance();
        hw               = Hardware.getInstance(hardwareMap);
        mecanumCommand   = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);
        turret.setMecanumCommand(mecanumCommand);
        // Turret gains: using tuned values from TurretMechanismTutorial defaults.
        // Teammate had kP=0.01, kD=0.001 here — removed so our values take effect.

        limelight = hw.limelight;
        limelight.pipelineSwitch(8);
        limelight.start();

        sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");
        colourSubsystem = new ColourSensorSubsystem(hardwareMap, hw, sorterSubsystem);

        intake   = hw.intake;
        pusher_R = hw.pusher_R;
        pusher_L = hw.pusher_L;
        leftLight  = hw.leftLight;
        rightLight = hw.rightLight;
        gate = hw.gate;

        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
        hw.sorter.setPosition(0.0);
        hw.leftLight.setPosition(0.0);
        hw.hood.setPosition(0.50);  // HOOD_MAX — resting position
        hw.rightLight.setPosition(1.0);
        gate.setPosition(0.6);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.llmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- PUSHER LOG INIT ----------------
        try {
            String ts = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            pusherLog = new BufferedWriter(new FileWriter("/sdcard/FIRST/pusher_log_" + ts + ".csv"), 16384);
            pusherLogEnabled = true;
            pusherLog.write("time_ms,loop_dt_ms,y_button,togglePusher,pusherReturning," +
                    "pusherReturnTimer_ms,pusher_R_cmd,pusher_L_cmd," +
                    "quickfire_state,quickfire_active," +
                    "pusher_source\n");
            pusherLog.flush();
        } catch (IOException e) { pusherLogEnabled = false; }
        totalTimer.reset();
        loopDtTimer.reset();

        // NOTE: Pinpoint heading NOT reset here — carries over from auto for correct FOD.
        // Use gamepad1.start to re-zero mid-match if needed.

        // ---------------- STATE ----------------
        boolean autoAimEnabled  = false;
        boolean prevA           = false;
        boolean previousXState  = false;
        boolean previousYState  = false;
        boolean togglePusher    = false;
        boolean isShooterOn     = false;
        boolean prevDpadLeft    = false;
        boolean prevManual      = false;
        boolean pusherReturning = false;
        boolean pusherAtFire    = false;  // latches true once RPM reached; prevents RPM oscillation pulling pusher back to preload   // true while waiting for pusher to physically return down

        waitForStart();

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {

            // ---------------- DRIVE (FIELD-CENTRIC) ----------------
            mecanumCommand.processOdometry();
            double heading = mecanumCommand.getOdoHeading(); // radians

            double inputY =  -gamepad1.left_stick_y;
            double inputX =   gamepad1.left_stick_x;
            double inputR =   gamepad1.right_stick_x;

            double fieldX = inputX * Math.cos(-heading) - inputY * Math.sin(-heading);
            double fieldY = inputX * Math.sin(-heading) + inputY * Math.cos(-heading);

            // Field rotation applied above via cos/sin — use normalMove (not fieldOrientedMove).
            theta = mecanumCommand.normalMove(fieldY, fieldX, inputR);

            // ---------------- LIMELIGHT ----------------
            llResult = limelight.getLatestResult();
            Double tx = (llResult != null && llResult.isValid()) ? llResult.getTx() : null;
            Double ty = (llResult != null && llResult.isValid()) ? llResult.getTy() : null;

            // ---------------- AUTO-AIM TOGGLE (A) ----------------
            boolean curA = gamepad1.a;
            if (curA && !prevA) {
                autoAimEnabled = !autoAimEnabled;
                leftLight.setPosition(autoAimEnabled ? 0.44 : 0.0);
            }
            prevA = curA;

            // ---------------- MANUAL TURRET OVERRIDE (bumpers) ----------------
            double manualPower = 0;
            int turretPos = hw.llmotor.getCurrentPosition();

            if (gamepad1.left_bumper) {
                double distToLimit = TURRET_MAX_TICKS - turretPos;
                double scale = Math.min(distToLimit / 50.0, 1.0);
                manualPower = (turretPos >= TURRET_MAX_TICKS) ? -0.3 : (0.15 + 0.2 * scale);
            } else if (gamepad1.right_bumper) {
                double distToLimit = turretPos - TURRET_MIN_TICKS;
                double scale = Math.min(distToLimit / 50.0, 1.0);
                manualPower = (turretPos <= TURRET_MIN_TICKS) ? 0.3 : -(0.15 + 0.2 * scale);
            }

            boolean isManual = (manualPower != 0);
            prevManual = isManual;

            // ---------------- TURRET CONTROL ----------------
            if (turretPos > TURRET_MAX_TICKS) {
                hw.llmotor.setPower(-0.3);
            } else if (turretPos < TURRET_MIN_TICKS) {
                hw.llmotor.setPower(0.3);
            } else if (isManual) {
                hw.llmotor.setPower(manualPower);
            } else if (autoAimEnabled) {
                if ((turretPos >= TURRET_MAX_TICKS && tx != null && tx < 0) ||
                        (turretPos <= TURRET_MIN_TICKS && tx != null && tx > 0)) {
                    hw.llmotor.setPower(0);
                } else {
                    turret.update(tx, ty);
                }
            } else {
                hw.llmotor.setPower(0);
            }

            if (autoAimEnabled && tx != null) {
                RobotLog.i(String.format(
                        "time:%.2f tx:%.1f pos:%d power:%.2f",
                        getRuntime(), tx, turretPos, hw.llmotor.getPower()));
            }

            // ---------------- COLOUR SENSOR ----------------
            // Updated before intake so isBallPresent() is current this loop.
            boolean isIntakeMotorOn  = gamepad1.right_trigger > 0.5;
            boolean isOuttakeMotorOn = gamepad1.left_trigger  > 0.5;
            if (isIntakeMotorOn) isOuttakeMotorOn = false; // intake wins
            colourSubsystem.update(isIntakeMotorOn);

            // ---------------- INTAKE (hold right trigger > 50%) ----------------
            if (isIntakeMotorOn)       intake.setPower(0.8);
            else if (isOuttakeMotorOn) intake.setPower(-0.8);
            else                       intake.setPower(0);

            gate.setPosition((isIntakeMotorOn || isOuttakeMotorOn) ? 0.7 : 0.6);

            // ---------------- SHOOTER TOGGLE (X) ----------------
            boolean curX = gamepad1.x;
            if (curX && !previousXState) {
                isShooterOn = !isShooterOn;
                if (isShooterOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    shooterSubsystem.spinup();
                } else {
                    shooterSubsystem.stopShooter();
                    leftLight.setPosition(autoAimEnabled ? 0.44 : 0.0);
                }
            }
            previousXState = curX;

            if (isShooterOn) {
                shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                shooterSubsystem.spinup();
                // Light: dim = spinning up, bright = locked on and RPM ready
                if (tx != null && Math.abs(tx) < 3 && shooterSubsystem.isRPMReached()) {
                    leftLight.setPosition(0.3);
                } else {
                    leftLight.setPosition(autoAimEnabled ? 0.44 : 0.0);
                }
            }

            // ---------------- PUSHER (hold Y = pre-load; fire when RPM ready) ----------------
            // Y held + flywheel not at speed → partial pre-load position (1/2 travel).
            //   Keeps the ball close to the shooter so it doesn't have to travel far on fire.
            //   Lower this fraction if 1/2 is still too high (e.g. try 0.4).
            // Y held + flywheel at speed → full fire position.
            // Y released → return to down, trigger sorter reverse.
            final double PRELOAD_FRACTION = 0.70;
            double preloadR = PusherConsts.PUSHER_DOWN_POSITION_R
                    + (PusherConsts.PUSHER_UP_POSITION_R - PusherConsts.PUSHER_DOWN_POSITION_R) * PRELOAD_FRACTION;
            double preloadL = PusherConsts.PUSHER_DOWN_POSITION_L
                    + (PusherConsts.PUSHER_UP_POSITION_L - PusherConsts.PUSHER_DOWN_POSITION_L) * PRELOAD_FRACTION;

            boolean curY = gamepad1.y;
            String pusherSource = "none";
            if (curY) {
                if (isShooterOn && (shooterSubsystem.isRPMReached() || pusherAtFire)) {
                    // RPM reached (or already committed to fire) — lock at full position.
                    // pusherAtFire latch prevents RPM oscillation pulling pusher back to preload.
                    pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                    togglePusher  = true;
                    pusherAtFire  = true;
                    pusherSource  = "Y_fire";
                } else {
                    // Waiting for speed — hold at pre-load
                    pusher_R.setPosition(preloadR);
                    pusher_L.setPosition(preloadL);
                    togglePusher = true;
                    pusherSource = "Y_preload";
                }
            } else {
                if (togglePusher) {
                    pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                    togglePusher  = false;
                    pusherAtFire  = false;  // reset latch on release
                    pusherReturning = true;
                    pusherReturnTimer.reset();
                    pusherSource  = "Y_release";
                }
            }
            previousYState = curY;

            // 400ms after pusher commanded down — physically back in place.
            // Step sorter backward one slot to bring next ball to shooter position.
            if (pusherReturning && pusherReturnTimer.milliseconds() >= 400) {
                pusherReturning = false;
                sorterSubsystem.manualSpinReverse();
            }

            // ---------------- SORTER MANUAL ----------------
            // B button: advance one slot forward (CW)
            if (gamepad1.b && sorterTimer.milliseconds() > 500) {
                sorterTimer.reset();
                sorterSubsystem.manualSpin();
            }

            // Back button: retreat one slot backward (CCW)
            if (gamepad1.back && sorterTimer.milliseconds() > 500) {
                sorterTimer.reset();
                sorterSubsystem.manualSpinReverse();
            }

            // ---------------- COLOUR SELECTION ----------------
            if (gamepad1.dpad_up) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.GREEN;
                rightLight.setPosition(0.5);
            }
            if (gamepad1.dpad_down) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.PURPLE;
                rightLight.setPosition(0.722);
            }
            // TODO: gamepad2 — move ANY colour selection to second controller
            // if (gamepad2.back) {
            //     sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.ANY;
            //     rightLight.setPosition(1.0);
            // }

            // ---------------- QUICKFIRE (dpad left/right) ----------------
            boolean quickfireWasActive = sorterSubsystem.isActive();
            if (gamepad1.dpad_left && !prevDpadLeft) sorterSubsystem.startQuickfire();
            if (sorterSubsystem.isActive()) {
                sorterSubsystem.quickfireState();
                // Quickfire internally commands pusher — flag it so the log shows the conflict
                if (quickfireWasActive) pusherSource = "quickfire:" + sorterSubsystem.quickfireState;
            }
            prevDpadLeft = gamepad1.dpad_left;
            if (gamepad1.dpad_right) sorterSubsystem.stopQuickfire();

            // ---------------- ODOMETRY RESET (start) ----------------
            if (gamepad1.start) mecanumCommand.resetPinPointOdometry();

            // ---------------- PUSHER LOG WRITE ----------------
            double dtMs = loopDtTimer.milliseconds();
            loopDtTimer.reset();
            if (pusherLogEnabled && pusherLog != null) {
                try {
                    pusherLogLine.setLength(0);
                    pusherLogLine.append(totalTimer.milliseconds()).append(',')
                            .append(dtMs).append(',')
                            .append(curY ? 1 : 0).append(',')
                            .append(togglePusher ? 1 : 0).append(',')
                            .append(pusherReturning ? 1 : 0).append(',')
                            .append(pusherReturnTimer.milliseconds()).append(',')
                            .append(pusher_R.getPosition()).append(',')
                            .append(pusher_L.getPosition()).append(',')
                            .append(sorterSubsystem.quickfireState).append(',')
                            .append(sorterSubsystem.isActive() ? 1 : 0).append(',')
                            .append(pusherSource).append('\n');
                    pusherLog.write(pusherLogLine.toString());
                    // Flush every 3s
                    if ((int)(totalTimer.milliseconds()) % 3000 < 50) pusherLog.flush();
                } catch (IOException e) { pusherLogEnabled = false; }
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Turret Ticks",       turretPos);
            telemetry.addData("Has Target",         turret.hasTarget());
            telemetry.addData("tx",                 tx);
            telemetry.addData("ty",                 ty);
            telemetry.addData("Distance",           turret.getDistanceTrack());
            telemetry.addData("Shooter RPM target", turret.getShootRPM());
            telemetry.addData("Auto Aim",           autoAimEnabled);
            telemetry.addData("Shooter on",         isShooterOn);
            telemetry.addData("Intake on",          isIntakeMotorOn);
            telemetry.addData("Outtake on",         isOuttakeMotorOn);
            telemetry.addLine("---------------------------------");
            telemetry.addData("Robot X",            mecanumCommand.getX());
            telemetry.addData("Robot Y",            mecanumCommand.getY());
            telemetry.addData("Heading (rad)",      heading);
            telemetry.addLine("---------------------------------");
            telemetry.addData("Red",    colourSubsystem.getRed());
            telemetry.addData("Green",  colourSubsystem.getGreen());
            telemetry.addData("Blue",   colourSubsystem.getBlue());
            telemetry.addData("Alpha",  colourSubsystem.getAlpha());
            telemetry.addData("Red2",   colourSubsystem.getRed2());
            telemetry.addData("Green2", colourSubsystem.getGreen2());
            telemetry.addData("Blue2",  colourSubsystem.getBlue2());
            telemetry.addData("Alpha2", colourSubsystem.getAlpha2());
            telemetry.addData("Last Red",   colourSubsystem.getLastValues()[0]);
            telemetry.addData("Last Green", colourSubsystem.getLastValues()[1]);
            telemetry.addData("Last Blue",  colourSubsystem.getLastValues()[2]);
            telemetry.addData("Last Alpha", colourSubsystem.getLastValues()[3]);
            telemetry.addData("Ball present", colourSubsystem.isBallPresent());
            telemetry.addLine("---------------------------------");
            telemetry.addData("Artifact count",   sorterSubsystem.getArtifactCount());
            telemetry.addData("Sorter contents",  Arrays.toString(sorterSubsystem.getSorterList()));
            telemetry.addData("Sorter position",  sorterSubsystem.getSorterPos());
            telemetry.addData("Selected colour",  sorterSubsystem.selectedColour);
            telemetry.update();
        }

        turret.closeLog();
        if (pusherLog != null) {
            try { pusherLog.flush(); pusherLog.close(); } catch (IOException ignored) {}
        }
    }
}