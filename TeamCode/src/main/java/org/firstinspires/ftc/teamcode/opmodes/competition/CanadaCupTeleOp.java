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

import java.util.Arrays;

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
    private Servo pusher_R, pusher_L, gate, light;

    private double theta;
    private double sorterPosition = 0.0;

    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    // -------------------------------------------------------------------------
    // Diagnostic state
    // -------------------------------------------------------------------------
    private double prevMotorPower  = 0;
    private double prevRawTx       = 0;
    private boolean prevHadTarget  = false;
    private double firstFrameTxJump = 0;
    private int    firstFrameTimer  = 0;   // loop-count countdown so reading stays visible

    // -------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);
        turret.setMecanumCommand(mecanumCommand);

        limelight = hw.limelight;
        limelight.pipelineSwitch(0);
        limelight.start();

        if (sorterSubsystem == null) {
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");
        }
        colourSubsystem = new ColourSensorSubsystem(hardwareMap, hw, sorterSubsystem);

        intake   = hw.intake;
        pusher_R = hw.pusher_R;
        pusher_L = hw.pusher_L;
        light    = hw.light;
        gate     = hw.gate;

        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
        hw.sorter.setPosition(0.0);
        hw.light.setPosition(0.0);
        gate.setPosition(0.6);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.llmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean autoAimEnabled   = false;
        boolean prevA            = false;
        boolean previousXState   = false, previousYState = false;
        boolean prevRightTrigger = false, prevLeftTrigger = false;
        boolean togglePusher     = false;
        boolean isIntakeMotorOn  = false, isOuttakeMotorOn = false, isShooterOn = false;
        boolean prevDpadLeft     = false;

        waitForStart();

        while (opModeIsActive()) {
            limelight.pipelineSwitch(8);
            mecanumCommand.processOdometry();

            // Snapshot values used in both control and telemetry
            double heading        = mecanumCommand.getOdoHeading();
            double headingDeg     = Math.toDegrees(heading);
            double robotRotVelDeg = mecanumCommand.getHeadingVelocity(); // deg/s
            double turretRelDeg   = hw.llmotor.getCurrentPosition() / 1.8;
            double turretPower    = hw.llmotor.getPower();

            double inputY = -gamepad1.left_stick_y;
            double inputX =  gamepad1.left_stick_x;
            double inputR =  gamepad1.right_stick_x;

            double fieldX = inputX * Math.cos(-heading) - inputY * Math.sin(-heading);
            double fieldY = inputX * Math.sin(-heading) + inputY * Math.cos(-heading);

            theta = mecanumCommand.normalMove(fieldY, fieldX, inputR);

            llResult = limelight.getLatestResult();
            Double tx = (llResult != null && llResult.isValid()) ? llResult.getTx() : null;
            Double ty = (llResult != null && llResult.isValid()) ? llResult.getTy() : null;
            boolean hasTarget = (tx != null);

            // Toggle Auto-Aim
            if (gamepad1.a && !prevA) {
                autoAimEnabled = !autoAimEnabled;
                light.setPosition(autoAimEnabled ? 0.44 : 0.0);
            }
            prevA = gamepad1.a;

            double manualPower = 0;
            if      (gamepad1.left_bumper)  manualPower =  0.4;
            else if (gamepad1.right_bumper) manualPower = -0.4;

            if (manualPower != 0) {
                hw.llmotor.setPower(manualPower);
            } else if (autoAimEnabled) {
                turret.update(tx, ty);
            } else {
                hw.llmotor.setPower(0);
            }

            // Re-read after update() so telemetry shows the power actually set
            turretPower = hw.llmotor.getPower();

            // --- Intake ---
            if (gamepad1.right_trigger > 0 && !prevRightTrigger) {
                isIntakeMotorOn = !isIntakeMotorOn;
                intake.setPower(isIntakeMotorOn ? 0.8 : 0);
                if (isIntakeMotorOn) isOuttakeMotorOn = false;
            }
            prevRightTrigger = gamepad1.right_trigger > 0;
            colourSubsystem.update(isIntakeMotorOn);

            if (gamepad1.left_trigger > 0 && !prevLeftTrigger) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
                intake.setPower(isOuttakeMotorOn ? -0.8 : 0);
                if (isOuttakeMotorOn) isIntakeMotorOn = false;
            }
            prevLeftTrigger = gamepad1.left_trigger > 0;

            gate.setPosition((isIntakeMotorOn || isOuttakeMotorOn) ? 0.7 : 0.6);

            // --- Shooter ---
            if (gamepad1.x && !previousXState) {
                isShooterOn = !isShooterOn;
                if (isShooterOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    shooterSubsystem.spinup();
                } else {
                    shooterSubsystem.stopShooter();
                    light.setPosition(0.0);
                }
            }
            previousXState = gamepad1.x;

            // --- Pusher ---
            if (gamepad1.y && !previousYState && !togglePusher) {
                pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                pusherTimer.reset();
                togglePusher = true;
            }
            previousYState = gamepad1.y;

            if (togglePusher && pusherTimer.milliseconds() >= 500) {
                pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                togglePusher = false;
            }

            // --- Sorter ---
            if (gamepad1.b && sorterTimer.milliseconds() > 500) {
                sorterPosition = (sorterPosition + 1) % 3;
                sorterTimer.reset();
                if      (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
                else if (sorterPosition == 1)   hw.sorter.setPosition(0.43);
                else                            hw.sorter.setPosition(0.875);
            }

            if (gamepad1.dpad_left && !prevDpadLeft) sorterSubsystem.startQuickfire();
            if (sorterSubsystem.isActive())          sorterSubsystem.quickfireState();
            prevDpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right) sorterSubsystem.stopQuickfire();

            if (gamepad1.start) mecanumCommand.resetPinPointOdometry();

            // =================================================================
            // DIAGNOSTIC TELEMETRY
            // Each section is labelled with which check it supports.
            // =================================================================

            double rawTxVal    = hasTarget ? tx : 0.0;
            double txFrameDelta = Math.abs(rawTxVal - prevRawTx);
            double powerDelta   = turretPower - prevMotorPower;
            // Expected feedforward power given current kFF & sign=-1
            double expectedFF  = -1.0 * robotRotVelDeg * 0.011;

            // Detect first frame target appears for stability gate check
            if (hasTarget && !prevHadTarget) {
                firstFrameTxJump = Math.abs(rawTxVal);
                firstFrameTimer  = 150; // stays visible for ~150 loops (~3 s at 50 Hz)
            }
            if (firstFrameTimer > 0) firstFrameTimer--;

            // ----- CHECK 1 & 2: Feedforward sign + magnitude -----
            // HOW TO TEST: auto-aim ON, no target visible, slowly rotate robot by hand.
            // GOOD: "FF Sign OK?" shows YES and turret visually stays still.
            // BAD sign: turret rotates WITH the robot instead of against it → flip kFF_SIGN.
            // BAD magnitude: turret still drifts despite correct sign → increase kFF (0.011).
            //                turret overshoots centre during spin → decrease kFF.
            telemetry.addLine("=== CHECK 1+2: FF Sign & Magnitude ===");
            telemetry.addData("Robot Rot Vel  (deg/s)", String.format("%.2f", robotRotVelDeg));
            telemetry.addData("Turret Power (actual)",  String.format("%.4f", turretPower));
            telemetry.addData("Expected FF Power",      String.format("%.4f", expectedFF));
            telemetry.addData("FF Sign OK?",
                    Math.abs(robotRotVelDeg) < 5.0
                            ? "spin robot to test"
                            : (Math.signum(turretPower) != Math.signum(robotRotVelDeg)
                            ? "YES - sign correct"
                            : "NO  - flip kFF_SIGN in subsystem"));

            // ----- CHECK 3: Filter alpha -----
            // HOW TO TEST: aim at a stationary target.
            // GOOD: tx delta is small and turret sits still on target.
            // High delta + jittery turret → lower TX_FILTER_ALPHA (currently 0.35).
            // Turret slow to acquire even with large tx → raise TX_FILTER_ALPHA.
            telemetry.addLine("=== CHECK 3: Filter Alpha ===");
            telemetry.addData("Raw tx          (deg)", hasTarget ? String.format("%.3f", tx) : "no target");
            telemetry.addData("tx frame delta  (deg)", String.format("%.3f", txFrameDelta));
            telemetry.addData("Filter verdict",
                    txFrameDelta > 1.5 ? "noisy  - consider lowering TX_FILTER_ALPHA" :
                            txFrameDelta < 0.1 ? "stable - if sluggish raise TX_FILTER_ALPHA" :
                                    "normal");

            // ----- CHECK 4: kP tuning -----
            // HOW TO TEST: drive toward/away from target while watching tx vs power.
            // Large tx, low power → kP too small.
            // Turret repeatedly crosses zero and hunts → kP too large (or kD too small).
            telemetry.addLine("=== CHECK 4: kP Tuning ===");
            telemetry.addData("tx error        (deg)", hasTarget ? String.format("%.3f", tx) : "—");
            telemetry.addData("kP",                    String.format("%.4f", turret.getkP()));
            telemetry.addData("P contribution  (est)", hasTarget
                    ? String.format("%.4f", -tx * turret.getkP()) : "—");
            telemetry.addData("Turret Rel      (deg)", String.format("%.2f", turretRelDeg));
            telemetry.addData("Turret World    (deg)", String.format("%.2f", headingDeg + turretRelDeg));

            // ----- CHECK 5: kD tuning -----
            // HOW TO TEST: watch power trend as turret settles onto target.
            // Power cuts then turret ticks past zero → raise kD.
            // Turret slows way too early, creeps last few degrees → lower kD.
            // Large per-frame power swings at rest → kD may be fighting noise, lower it.
            telemetry.addLine("=== CHECK 5: kD Tuning ===");
            telemetry.addData("kD",                    String.format("%.4f", turret.getkD()));
            telemetry.addData("Motor power delta/loop", String.format("%.4f", powerDelta));
            telemetry.addData("kD verdict",
                    Math.abs(powerDelta) > 0.15 ? "large swing  - raise kD or check noise" :
                            Math.abs(powerDelta) < 0.005 ? "very smooth  - kD OK (watch overshoot)" :
                                    "moderate     - watch settle behaviour");

            // ----- CHECK 6: Stability gate -----
            // HOW TO TEST: drive until target leaves frame, re-enter, observe "first-appear tx".
            // Large first-appear tx + turret lunges → lower TX_STABILITY_THRESHOLD_DEG (0.8).
            // Turret hesitates before it starts tracking → raise TX_STABILITY_THRESHOLD_DEG.
            telemetry.addLine("=== CHECK 6: Stability Gate ===");
            telemetry.addData("Target visible",          hasTarget);
            telemetry.addData("First-appear tx    (deg)",
                    firstFrameTimer > 0
                            ? String.format("%.2f  (%d loops ago)", firstFrameTxJump, 150 - firstFrameTimer)
                            : "lose + reacquire target to test");
            telemetry.addData("Gate verdict",
                    firstFrameTimer > 0
                            ? (firstFrameTxJump > 5.0 ? "large - lower TX_STABILITY_THRESHOLD_DEG" :
                            firstFrameTxJump < 1.0 ? "small - threshold OK" :
                                    "moderate - watch for lunge")
                            : "—");

            // ----- General -----
            telemetry.addLine("=== General ===");
            telemetry.addData("Auto Aim",     autoAimEnabled);
            telemetry.addData("Heading (deg)",String.format("%.2f", headingDeg));
            telemetry.addData("Shooter RPM",  String.format("%.0f", turret.getShootRPM()));
            telemetry.addData("Distance (m)", String.format("%.3f", turret.getDistanceTrack()));

            // Save state for next loop
            prevMotorPower = turretPower;
            prevRawTx      = rawTxVal;
            prevHadTarget  = hasTarget;

            telemetry.update();
        }
    }
}