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
<<<<<<< HEAD
    private DcMotor shooter;
    private Servo pusher_R;
    private Servo pusher_L;
    private Servo gate;
    private Servo leftLight;
    private Servo rightLight;
=======
    private Servo pusher_R, pusher_L, gate, light;

>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36
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

        sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");

        sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.ANY;

        colourSubsystem = new ColourSensorSubsystem(hardwareMap, hw, sorterSubsystem);

        intake   = hw.intake;
        pusher_R = hw.pusher_R;
        pusher_L = hw.pusher_L;
<<<<<<< HEAD
        leftLight = hw.leftLight;
        rightLight = hw.rightLight;
        gate = hw.gate;
=======
        light    = hw.light;
        gate     = hw.gate;
>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36

        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
        hw.sorter.setPosition(0.0);
        hw.leftLight.setPosition(0.0);
        hw.rightLight.setPosition(1.0);
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

<<<<<<< HEAD
        boolean previousXState = false;
        boolean previousYState = false;
        boolean prevRightTrigger = false;
        boolean prevLeftTrigger = false;
        boolean togglePusher = false;
        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean isShooterOn = false;
        boolean prevDpadLeft = false;
        boolean prevManual = false;
        boolean attemptedFullIntake = false;

        // ---------------- MAIN CONTROL LOOP ----------------
=======
>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36
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
<<<<<<< HEAD
                if (autoAimEnabled) {
                    leftLight.setPosition(0.44);
                } else {
                    leftLight.setPosition(0.0);
                }
=======
                light.setPosition(autoAimEnabled ? 0.44 : 0.0);
>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36
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
<<<<<<< HEAD

                if (sorterSubsystem.getArtifactCount() == 3) {
                    attemptedFullIntake = true;
                } else {
                    attemptedFullIntake = false;
                }

                if (isIntakeMotorOn) {
                    isOuttakeMotorOn = false;
                    intake.setPower(0.8);
                } else {
                    intake.setPower(0);
                }
            }
            if (sorterSubsystem.getArtifactCount() == 3 && isIntakeMotorOn && !attemptedFullIntake) {
                isIntakeMotorOn = false;
                intake.setPower(0);
            }

            prevRightTrigger = curRightTrigger;
=======
                intake.setPower(isIntakeMotorOn ? 0.8 : 0);
                if (isIntakeMotorOn) isOuttakeMotorOn = false;
            }
            prevRightTrigger = gamepad1.right_trigger > 0;
>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36
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
                    leftLight.setPosition(0.0);
                }
            }
            previousXState = gamepad1.x;

<<<<<<< HEAD
            if (isShooterOn && tx != null && Math.abs(tx) < 3) {
                if (shooterSubsystem.isRPMReached()) {
                    leftLight.setPosition(0.3);
                } else {
                    leftLight.setPosition(0.0);
                }
=======
            // --- Pusher ---
            if (gamepad1.y && !previousYState && !togglePusher) {
                pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                pusherTimer.reset();
                togglePusher = true;
>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36
            }
            previousYState = gamepad1.y;

            if (togglePusher && pusherTimer.milliseconds() >= 500) {
                pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                togglePusher = false;
            }

            // --- Sorter ---
            if (gamepad1.b && sorterTimer.milliseconds() > 500) {
//                sorterPosition = (sorterPosition + 1) % 3;
//                sorterTimer.reset();
//                if (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
//                else if (sorterPosition == 1) hw.sorter.setPosition(0.43);
//                else hw.sorter.setPosition(0.875);
                sorterTimer.reset();
<<<<<<< HEAD
                sorterSubsystem.manualSpin();
            }

            // ---------------- COLOUR SELECTION ----------------
            if (gamepad1.dpad_up) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.GREEN;
                hw.rightLight.setPosition(0.5);
            }

            if (gamepad1.dpad_down) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.PURPLE;
                hw.rightLight.setPosition(0.722);
            }

            if (gamepad1.back) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.ANY;
                hw.rightLight.setPosition(1.0);
=======
                if      (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
                else if (sorterPosition == 1)   hw.sorter.setPosition(0.43);
                else                            hw.sorter.setPosition(0.875);
>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36
            }

            if (gamepad1.dpad_left && !prevDpadLeft) sorterSubsystem.startQuickfire();
            if (sorterSubsystem.isActive())          sorterSubsystem.quickfireState();
            prevDpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right) sorterSubsystem.stopQuickfire();

            if (gamepad1.start) mecanumCommand.resetPinPointOdometry();

            // =================================================================
            // DIAGNOSTIC TELEMETRY
            // =================================================================

<<<<<<< HEAD
            // ---------------- TELEMETRY ----------------
            if (llResult != null && llResult.isValid()) {
                telemetry.addData("Tag Detected", "ID: " + llResult.getFiducialResults().get(0).getFiducialId());
            } else {
                telemetry.addData("Tag Detected", "None");
            }
            telemetry.addData("Heading Delta", turret.getDebugHeadingDelta());
            telemetry.addData("Target Ticks", turret.getDebugTargetTicks());
            telemetry.addData("Turret Ticks", hw.llmotor.getCurrentPosition());
            telemetry.addData("Has Target", turret.hasTarget());
            telemetry.addData("Y state", currentYState);
            telemetry.addData("Target Visible", tx != null);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("distance", turret.getDistanceTrack());
            telemetry.addData("Turret kP", turret.getkP());
            telemetry.addData("Turret kD", turret.getkD());
            telemetry.addData("Shooter RPM", turret.getShootRPM());
            telemetry.addData("Intake On", isIntakeMotorOn);
            telemetry.addData("Outtake On", isOuttakeMotorOn);
            telemetry.addLine("---------------------------------");
            telemetry.addData("Robot X", mecanumCommand.getX());
            telemetry.addData("Robot Y", mecanumCommand.getY());
            telemetry.addData("Heading (rad)", heading);
            telemetry.addData("Auto Aim Enabled", autoAimEnabled);
            telemetry.addData("Manual Override", curLeftBumper || curRightBumper);
            telemetry.addData("Red", colourSubsystem.getRed());
            telemetry.addData("Green", colourSubsystem.getGreen());
            telemetry.addData("Blue", colourSubsystem.getBlue());
            telemetry.addData("Alpha", colourSubsystem.getAlpha());
            telemetry.addData("Red2", colourSubsystem.getRed2());
            telemetry.addData("Green2", colourSubsystem.getGreen2());
            telemetry.addData("Blue2", colourSubsystem.getBlue2());
            telemetry.addData("Alpha2", colourSubsystem.getAlpha2());
            telemetry.addData("Detected Count", sorterSubsystem.getArtifactCount());
            telemetry.addData("Current Balls", Arrays.toString(sorterSubsystem.getSorterList()));
            telemetry.addData("Current Sorter Position", sorterSubsystem.getSorterPos());
=======
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

>>>>>>> 486e14bba0f2df185237fe2761660932bad45b36
            telemetry.update();
        }

        // Flush and close the turret CSV log
        turret.closeLog();
    }
}