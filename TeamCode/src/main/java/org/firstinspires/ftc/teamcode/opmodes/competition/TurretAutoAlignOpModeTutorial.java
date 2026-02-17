/**
 * TurretAutoAlignOpModeTutorial
 *
 * This OpMode demonstrates automatic turret alignment using Limelight.
 *
 * Responsibilities:
 * - Read Limelight vision data
 * - Compute distance to target
 * - Adjust hood position
 * - Send tx error to turret subsystem
 *
 * The turret control logic itself lives in TurretMechanismTutorial.
 *
 * This separation allows turret alignment to be reused in:
 * - Autonomous
 * - Future robots
 * - Different vision pipelines
 */

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
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismTutorial;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

@TeleOp(name = "TurretAutoAlignOpModeTutorial", group = "TeleOp")
public class TurretAutoAlignOpModeTutorial extends LinearOpMode {

    // ---------------- SUBSYSTEMS ----------------
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private ShooterSubsystem shooterSubsystem;
    private SorterSubsystem sorterSubsystem;
    private TurretMechanismTutorial turret;

    // ---------------- LIMELIGHT ----------------
    private Limelight3A limelight;
    private LLResult llResult;

    // ---------------- HARDWARE ----------------
    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private Servo hood;

    private double theta;
    private double hoodPos = 0.846;
    private double sorterPosition = 0.0;
    private double shootSpeed = 4800;

    // ---------------- TIMERS ----------------
    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    // ---------------- HOOD AUTO-ADJUST CONSTANTS ----------------
    private final double LIMELIGHT_HEIGHT = 0.35; // meters, height of Limelight camera
    private final double LIMELIGHT_ANGLE = Math.toRadians(25); // radians, mounting angle of Limelight
    private final double TARGET_HEIGHT = 0.9; // meters, center of AprilTag
    private final double HOOD_MIN = 0.359;
    private final double HOOD_MAX = 0.846;
    private final double MIN_DISTANCE = 0.5; // meters, closest distance to target
    private final double MAX_DISTANCE = 3.0; // meters, farthest distance to target

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- INITIALIZATION ----------------
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        turret = new TurretMechanismTutorial();

        // Initialize turret PID
        turret.init(hardwareMap);
        turret.setkP(0.035);  // Tuned PID
        turret.setkD(0.001);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); //uses pipeline 8 becuase this is where the blue alliance tag 20 lives
        limelight.start();

        // Initialize hardware
        intake = hw.intake;
        shooter = hw.shooter;
        hood = hw.hood;
        pusher = hw.pusher;
        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.sorter.setPosition(0.0);
        hw.hood.setPosition(hoodPos);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        if (sorterSubsystem == null) {
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");
        }

        waitForStart();
        turret.resetTimer();

        // ---------------- OPLOOP VARIABLES ----------------
        boolean previousXState = false;
        boolean previousYState = false;
        boolean prevRightTrigger = false;
        boolean prevLeftTrigger = false;
        boolean curRightTrigger;
        boolean curLeftTrigger;
        boolean currentXState;
        boolean currentYState;
        boolean curRB;
        boolean curLB;
        boolean togglePusher = false;
        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;

        while (opModeIsActive()) {

            // ---------------- DRIVE ----------------
            mecanumCommand.processOdometry();
            theta = mecanumCommand.normalMove(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            // ---------------- TURRET AUTO-ALIGN ----------------
            llResult = limelight.getLatestResult();
            Double tx = null;
            Double distance = null; // Distance to target

            if (llResult != null && llResult.isValid()) {
                tx = llResult.getTx(); // horizontal offset for turret
                double ty = llResult.getTy(); // vertical offset

                // ---- calculate approximate distance to AprilTag ----
                distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                        Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));

                // ---- map distance to hood position ----
                hoodPos = HOOD_MIN + (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE) * (HOOD_MAX - HOOD_MIN);
                hoodPos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPos)); // clamp
                hood.setPosition(hoodPos);
            }

            // Update turret
            turret.update(tx);

            // ---------------- INTAKE ----------------
            curRightTrigger = gamepad1.right_trigger > 0;
            if (curRightTrigger && !prevRightTrigger) {
                isIntakeMotorOn = !isIntakeMotorOn;
                intake.setPower(isIntakeMotorOn ? 0.8 : 0);
            }
            prevRightTrigger = curRightTrigger;

            curLeftTrigger = gamepad1.left_trigger > 0;
            if (curLeftTrigger && !prevLeftTrigger) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
                intake.setPower(isOuttakeMotorOn ? -0.8 : 0);
            }
            prevLeftTrigger = curLeftTrigger;

            // ---------------- SHOOTER ----------------
            currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
                if (isOuttakeMotorOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(shootSpeed));
                    shooterSubsystem.spinup();
                } else {
                    shooterSubsystem.stopShooter();
                }
            }
            previousXState = currentXState;

            // ---------------- SORTER ----------------
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            if (up) {
                shootSpeed = Math.min(6000.0, shootSpeed + 30.0);
                sleep(500);
            }
            if (down) {
                shootSpeed = Math.max(0.0, shootSpeed - 30.0);
                sleep(500);
            }

            if (gamepad1.b && sorterTimer.milliseconds() > 1000) {
                sorterPosition = (sorterPosition + 1) % 3;
                sorterTimer.reset();
                if (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
                else if (sorterPosition == 1) hw.sorter.setPosition(0.43);
                else if (sorterPosition == 2) hw.sorter.setPosition(0.875);
            }

            // ---------------- PUSHER ----------------
            currentYState = gamepad1.y;
            if (currentYState && !previousYState) {
                if (!togglePusher) {
                    pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
                    pusherTimer.reset();
                    togglePusher = true;
                }
            }
            previousYState = currentYState;

            if (togglePusher && pusherTimer.milliseconds() >= 500) {
                pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
                togglePusher = false;
            }

            // ---------------- RESET ODOMETRY ----------------
            if (gamepad1.start) {
                mecanumCommand.resetPinPointOdometry();
            }

            // ---------------- TELEMETRY ----------------
            if (tx != null) {
                telemetry.addData("Target Visible", true);
                telemetry.addData("tx", tx);
                telemetry.addData("Distance (m)", distance);
            } else {
                telemetry.addLine("No Target Detected - Turret Stopped");
            }

            telemetry.addData("Turret kP", turret.getkP());
            telemetry.addData("Turret kD", turret.getkD());
            telemetry.addData("Hood pos", hoodPos);
            telemetry.addData("Shooter RPM", shootSpeed);
            telemetry.addData("Intake On", isIntakeMotorOn);
            telemetry.addData("Outtake On", isOuttakeMotorOn);
            telemetry.addLine("---------------------------------");
            telemetry.addData("X", mecanumCommand.getX());
            telemetry.addData("Y", mecanumCommand.getY());
            telemetry.addData("Theta", mecanumCommand.getOdoHeading());
            telemetry.update();
        }
    }
}
