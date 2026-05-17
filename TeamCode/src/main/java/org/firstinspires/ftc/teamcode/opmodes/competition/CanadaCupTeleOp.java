package org.firstinspires.ftc.teamcode.opmodes.competition;

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
    private int    firstFrameTimer  = 0;

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

        // OPTIMIZATION: Initialize the Limelight directly to pipeline 8.
        // This avoids calling pipelineSwitch inside the loop which choked loop times.
        limelight = hw.limelight;
        limelight.pipelineSwitch(8);
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
            // OPTIMIZATION: Removed limelight.pipelineSwitch(8) from here to prevent blocking hardware calls.
            mecanumCommand.processOdometry();

            // Snapshot values used in both control and telemetry
            double heading        = mecanumCommand.getOdoHeading();
            double headingDeg     = Math.toDegrees(heading);
            double robotRotVelDeg = mecanumCommand.getHeadingVelocity();
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
            // =================================================================

            double rawTxVal     = hasTarget ? tx : 0.0;
            double txFrameDelta = Math.abs(rawTxVal - prevRawTx);
            double powerDelta   = turretPower - prevMotorPower;

            // Detect first frame target appears
            if (hasTarget && !prevHadTarget) {
                firstFrameTxJump = Math.abs(rawTxVal);
                firstFrameTimer  = 150;
            }
            if (firstFrameTimer > 0) firstFrameTimer--;

            // ----- Turret status (most important — always visible) -----
            telemetry.addLine("=== TURRET STATUS ===");
            telemetry.addData("Auto Aim",          autoAimEnabled);
            telemetry.addData("Has Target",        turret.hasTarget());
            telemetry.addData("LL reports target", hasTarget);
            telemetry.addData("tx (raw)",          hasTarget ? String.format("%.2f°", tx) : "none");
            telemetry.addData("Error (turret)",    String.format("%.2f°", rawTxVal));
            telemetry.addData("Motor power",       String.format("%.3f", turretPower));
            telemetry.addData("Turret pos",        String.format("%.1f°", turretRelDeg));

            // ----- Stale detection (new — catches false LL detections) -----
            telemetry.addLine("=== STALE DETECTION ===");
            telemetry.addData("tx frame delta",    String.format("%.3f°", txFrameDelta));
            telemetry.addData("Stale?",
                    hasTarget && !turret.hasTarget() ? "YES — LL rejected (frozen tx)"
                            : hasTarget ? "no — tracking normally"
                            : "— (no LL target)");

            // ----- PID check -----
            telemetry.addLine("=== PID ===");
            double estP = hasTarget ? turret.getkP() * rawTxVal : 0.0;
            telemetry.addData("kP (linear)",       String.format("%.3f", turret.getkP()));
            telemetry.addData("kD",               String.format("%.4f", turret.getkD()));
            telemetry.addData("kI",               String.format("%.4f", turret.getkI()));
            telemetry.addData("Est. P-term",      String.format("%.3f", estP));
            telemetry.addData("Power delta/loop",  String.format("%.4f", powerDelta));

            // ----- Stability gate -----
            telemetry.addLine("=== STABILITY GATE ===");
            telemetry.addData("First-appear tx",
                    firstFrameTimer > 0
                            ? String.format("%.2f° (%d loops ago)", firstFrameTxJump, 150 - firstFrameTimer)
                            : "lose + reacquire to test");

            // ----- General -----
            telemetry.addLine("=== GENERAL ===");
            telemetry.addData("Heading",      String.format("%.1f°", headingDeg));
            telemetry.addData("Shooter RPM",  String.format("%.0f", turret.getShootRPM()));
            telemetry.addData("Distance",     String.format("%.2f m", turret.getDistanceTrack()));

            // Save state for next loop
            prevMotorPower = turretPower;
            prevRawTx      = rawTxVal;
            prevHadTarget  = hasTarget;

            telemetry.update();
        }

        // Flush and close the turret CSV log
        turret.closeLog();
    }
}