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
    private Servo pusher_R, pusher_L, gate, leftLight, rightLight;

    private double theta;
    private double sorterPosition = 0.0;

    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    // -------------------------------------------------------------------------
    // Diagnostic state
    // -------------------------------------------------------------------------
    private double  prevMotorPower   = 0;
    private double  prevRawTx        = 0;
    private boolean prevHadTarget    = false;
    private double  firstFrameTxJump = 0;
    private int     firstFrameTimer  = 0;

    // -------------------------------------------------------------------------

    /** Sets both lights to the same position. */
    private void setLights(double position) {
        leftLight.setPosition(position);
        rightLight.setPosition(position);
    }

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand   = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);
        turret.setMecanumCommand(mecanumCommand);

        limelight = hw.limelight;
        limelight.pipelineSwitch(8);
        limelight.start();

        if (sorterSubsystem == null) {
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");
        }
        colourSubsystem = new ColourSensorSubsystem(hardwareMap, hw, sorterSubsystem);

        intake     = hw.intake;
        pusher_R   = hw.pusher_R;
        pusher_L   = hw.pusher_L;
        leftLight  = hw.leftLight;
        rightLight = hw.rightLight;
        gate       = hw.gate;

        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
        hw.sorter.setPosition(0.0);
        setLights(0.0);
        gate.setPosition(0.6);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.llmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean autoAimEnabled   = false;
        boolean prevA            = false;
        boolean previousXState   = false, previousYState = false;
        boolean togglePusher     = false;
        boolean isShooterOn = false;
        boolean prevDpadLeft         = false;
        int     loopCount            = 0;
        boolean attemptedFullIntake  = false;
        boolean pusherFiredThisFrame = false;
        double  batteryV             = 0;
        final com.qualcomm.robotcore.util.ElapsedTime opTimer = new com.qualcomm.robotcore.util.ElapsedTime();
        final StringBuilder sb = new StringBuilder(64);  // reused for telemetry to reduce GC

        // JVM warmup — force class loading and JIT compilation before the match starts
        // so the first loop doesn't freeze for 1+ seconds loading everything at once
        {
            StringBuilder warmup = new StringBuilder(512);
            warmup.append(0.0).append(',').append(true).append(',').append(1);
            double dummy = Math.tan(0.1) + Math.atan(0.1) + Math.pow(0.5, 1.5);
            Runtime.getRuntime().freeMemory();
            telemetry.addData("Status", "Ready");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            mecanumCommand.processOdometry();
            pusherFiredThisFrame = false;
            loopCount++;
            // Battery and limelight staleness for diagnostic logging
            try { batteryV = hardwareMap.voltageSensor.iterator().next().getVoltage(); } catch (Exception e) { batteryV = 0; }
            long llStaleMs = (llResult != null) ? llResult.getStaleness() : -1;

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
                if (!autoAimEnabled) setLights(0.0);
            }
            prevA = gamepad1.a;

            double manualPower = 0;
            if      (gamepad1.left_bumper)  manualPower =  0.4;
            else if (gamepad1.right_bumper) manualPower = -0.4;

            if (manualPower != 0) {
                turret.setManualPower(manualPower);
            } else if (autoAimEnabled) {
                turret.setAutoMode();
            } else {
                // Auto-aim off and no bumper held — hold turret still.
                // Must use setManualPower(0) not setAutoMode(), otherwise
                // turret.update() runs the PID and fights the idle state.
                turret.setManualPower(0);
            }

            turretPower = hw.llmotor.getPower();

            // -----------------------------------------------------------------
            // Shooter — toggle on X, update RPM and spin every loop
            // -----------------------------------------------------------------
            if (gamepad1.x && !previousXState) {
                isShooterOn = !isShooterOn;
                if (!isShooterOn) {
                    shooterSubsystem.stopShooter();
                    setLights(autoAimEnabled ? 0.44 : 0.0);
                }
            }
            previousXState = gamepad1.x;

            // Ready-to-fire: shooter on, RPM within tolerance, turret has lock
            // Also gate on turret error — don't fire if turret is oscillating through target
            boolean readyToFire = isShooterOn
                    && shooterSubsystem.isRPMReached()
                    && turret.hasTarget()
                    && Math.abs(turret.getLastError()) < 4.0;

            if (isShooterOn) {
                // RPM target is only updated when Y is pressed (see pusher block below)
                // — keeps flywheel stable at one speed rather than chasing distance changes
                shooterSubsystem.spinup();
                setLights(readyToFire ? 0.44 : 0.22);
            }

            // -----------------------------------------------------------------
            // Intake — hold right trigger to intake, hold left trigger to outtake.
            // Auto-stops intake when sorter is full (3 artifacts).
            // -----------------------------------------------------------------
            boolean isIntakeMotorOn  = gamepad1.right_trigger > 0;
            boolean isOuttakeMotorOn = gamepad1.left_trigger  > 0;

            if (sorterSubsystem.getArtifactCount() == 3) {
                attemptedFullIntake = true;
            } else {
                attemptedFullIntake = false;
            }
            // Stop intake automatically when full
            if (isIntakeMotorOn && sorterSubsystem.getArtifactCount() == 3 && !attemptedFullIntake) {
                isIntakeMotorOn = false;
            }

            if (isIntakeMotorOn) {
                intake.setPower(0.8);
            } else if (isOuttakeMotorOn) {
                intake.setPower(-0.8);
            } else {
                intake.setPower(0);
            }

            colourSubsystem.update(isIntakeMotorOn);
            gate.setPosition((isIntakeMotorOn || isOuttakeMotorOn) ? 0.7 : 0.6);

            // -----------------------------------------------------------------
            // While Y is held: continuously update RPM from current distance.
            // When Y is released the last measured target is held — flywheel
            // stays at that speed until Y is pressed again.
            if (gamepad1.y && !togglePusher) {
                shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
            }

            // Pusher — Y held moves to 2/3 pre-load position.
            // Fires all the way once flywheel reaches the snapshotted RPM target.
            double partialR = PusherConsts.PUSHER_DOWN_POSITION_R
                    + (PusherConsts.PUSHER_UP_POSITION_R - PusherConsts.PUSHER_DOWN_POSITION_R) * (2.0 / 3.0);
            double partialL = PusherConsts.PUSHER_DOWN_POSITION_L
                    + (PusherConsts.PUSHER_UP_POSITION_L - PusherConsts.PUSHER_DOWN_POSITION_L) * (2.0 / 3.0);

            if (!togglePusher) {
                if (gamepad1.y && readyToFire) {
                    // Flywheel ready — fire all the way
                    pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                    pusherTimer.reset();
                    togglePusher = true;
                    pusherFiredThisFrame = true;
                } else if (gamepad1.y) {
                    // Y held but not ready — hold at 2/3 pre-load position
                    pusher_R.setPosition(partialR);
                    pusher_L.setPosition(partialL);
                } else {
                    // Y released — return to down
                    pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                }
            }
            previousYState = gamepad1.y;

            if (togglePusher && pusherTimer.milliseconds() >= 500) {
                pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                togglePusher = false;
            }

            // Update turret after pusher block so pusherFiredThisFrame is correct in log.
            // Always called — turret.update() handles its own manual/auto/idle logic
            // and must run every loop to keep logging and odometry current.
            turret.update(tx, ty, shooterSubsystem.getShooterVelocity() * 60.0 / 28.0,
                    turret.getShootRPM(), shooterSubsystem.isRPMReached(), pusherFiredThisFrame,
                    batteryV, llStaleMs,
                    mecanumCommand.getOdoX(), mecanumCommand.getOdoY(),
                    mecanumCommand.getHeadingVelocity(), autoAimEnabled);


            // -----------------------------------------------------------------
            // Sorter
            // -----------------------------------------------------------------
            if (gamepad1.b && sorterTimer.milliseconds() > 500) {
                sorterTimer.reset();
                sorterSubsystem.manualSpin();
            }

            // Colour selection — dpad up: GREEN, dpad down: PURPLE, back: ANY
            if (gamepad1.dpad_up) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.GREEN;
                rightLight.setPosition(0.5);
            }
            if (gamepad1.dpad_down) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.PURPLE;
                rightLight.setPosition(0.722);
            }
            if (gamepad1.back) {
                sorterSubsystem.selectedColour = SorterSubsystem.SelectedColour.ANY;
                rightLight.setPosition(1.0);
            }

            if (gamepad1.dpad_left && !prevDpadLeft) sorterSubsystem.startQuickfire();
            if (sorterSubsystem.isActive())          sorterSubsystem.quickfireState();
            prevDpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right) sorterSubsystem.stopQuickfire();

            if (gamepad1.start) mecanumCommand.resetPinPointOdometry();

            // =================================================================
            // DIAGNOSTIC TELEMETRY
            // =================================================================

            double rawTxVal = hasTarget ? tx : 0.0;

// Telemetry throttled to every 3 loops — reduces String allocation by 66%
            if (loopCount % 3 == 0) {
                telemetry.addLine("=== TURRET STATUS ===");
                telemetry.addData("Auto Aim",     autoAimEnabled);
                telemetry.addData("Has Target",   turret.hasTarget());
                telemetry.addData("tx (raw)",     hasTarget ? ((int)(tx*100)/100.0) + "°" : "none");
                telemetry.addData("Error",        (int)(rawTxVal*10)/10.0 + "°");
                telemetry.addData("Motor power",  (int)(turretPower*1000)/1000.0);
                telemetry.addData("Turret pos",   (int)(turretRelDeg*10)/10.0 + "°");

                telemetry.addLine("=== SHOOTER ===");
                telemetry.addData("Shooter on",   isShooterOn);
                telemetry.addData("Target RPM",   (int) turret.getShootRPM());
                telemetry.addData("RPM ready",    shooterSubsystem.isRPMReached());
                telemetry.addData("Ready to fire",readyToFire);
                telemetry.addData("Distance",     (int)(turret.getDistanceTrack()*100)/100.0 + "m");

                telemetry.addLine("=== DIAGNOSTICS ===");
                telemetry.addData("Battery",      (int)(batteryV*100)/100.0 + "V");
                telemetry.addData("LL staleness", llStaleMs + "ms");
                telemetry.addData("Heap free",    (int)(Runtime.getRuntime().freeMemory()/1048576.0) + "MB");
                telemetry.addData("Heading",      (int)(headingDeg*10)/10.0 + "°");

                telemetry.addLine("=== SORTER ===");
                telemetry.addData("Artifact count",  sorterSubsystem.getArtifactCount());
                telemetry.addData("Selected colour", sorterSubsystem.selectedColour);
                telemetry.addData("Sorter pos",      sorterSubsystem.getSorterPos());
                telemetry.update();
            }
        }

        turret.closeLog();
    }
}