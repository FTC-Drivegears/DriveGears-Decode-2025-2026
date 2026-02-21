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

/**
 * TurretAutoAlignOpModeTutorial
 *
 * Demonstrates automatic turret alignment using Limelight vision.
 *
 * System architecture:
 * OpMode responsibilities:
 * - Read driver input
 * - Read Limelight data
 * - Call subsystem updates
 *
 * Turret subsystem responsibilities:
 * - PD alignment using tx
 * - Distance estimation using ty
 * - Hood auto-adjustment
 *
 * This separation keeps robot logic modular and easier to maintain.
 */
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

    private double theta;
    private double sorterPosition = 0.0;
    private double shootSpeed = 4800;

    // ---------------- TIMERS ----------------
    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- INITIALIZATION ----------------
        hw = Hardware.getInstance(hardwareMap);

        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);

        // Turret tuning values (adjust during testing)
        turret.setkP(0.035);
        turret.setkD(0.001);

        // ---------------- LIMELIGHT SETUP ----------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        // ---------------- HARDWARE SETUP ----------------
        intake = hw.intake;
        shooter = hw.shooter;
        pusher = hw.pusher;

        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.sorter.setPosition(0.0);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        if (sorterSubsystem == null) {
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");
        }

        waitForStart();
        turret.resetTimer();

        // ---------------- STATE VARIABLES ----------------
        boolean previousXState = false;
        boolean previousYState = false;
        boolean prevRightTrigger = false;
        boolean prevLeftTrigger = false;

        boolean togglePusher = false;
        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;

        // ---------------- MAIN CONTROL LOOP ----------------
        while (opModeIsActive()) {

            // ---------------- DRIVE ----------------
            mecanumCommand.processOdometry();
            theta = mecanumCommand.normalMove(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            // ---------------- LIMELIGHT + TURRET ----------------
            llResult = limelight.getLatestResult();

            Double tx = null;
            Double ty = null;

            if (llResult != null && llResult.isValid()) {
                tx = llResult.getTx();
                ty = llResult.getTy();
            }

            // Turret subsystem performs alignment and hood adjustment
            turret.update(tx, ty);

            // ---------------- INTAKE TOGGLE ----------------
            boolean curRightTrigger = gamepad1.right_trigger > 0;
            if (curRightTrigger && !prevRightTrigger) {
                isIntakeMotorOn = !isIntakeMotorOn;
                intake.setPower(isIntakeMotorOn ? 0.8 : 0);
            }
            prevRightTrigger = curRightTrigger;

            // ---------------- OUTTAKE TOGGLE ----------------
            boolean curLeftTrigger = gamepad1.left_trigger > 0;
            if (curLeftTrigger && !prevLeftTrigger) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
                intake.setPower(isOuttakeMotorOn ? -0.8 : 0);
            }
            prevLeftTrigger = curLeftTrigger;

            // ---------------- SHOOTER TOGGLE ----------------
            boolean currentXState = gamepad1.x;
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

            // ---------------- SHOOT SPEED ADJUST ----------------
            if (gamepad1.dpad_up) {
                shootSpeed = Math.min(6000.0, shootSpeed + 30.0);
                sleep(300);
            }

            if (gamepad1.dpad_down) {
                shootSpeed = Math.max(0.0, shootSpeed - 30.0);
                sleep(300);
            }

            // ---------------- SORTER CONTROL ----------------
            if (gamepad1.b && sorterTimer.milliseconds() > 1000) {
                sorterPosition = (sorterPosition + 1) % 3;
                sorterTimer.reset();

                if (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
                else if (sorterPosition == 1) hw.sorter.setPosition(0.43);
                else hw.sorter.setPosition(0.875);
            }

            // ---------------- PUSHER CONTROL ----------------
            boolean currentYState = gamepad1.y;
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

            // ---------------- ODOMETRY RESET ----------------
            if (gamepad1.start) {
                mecanumCommand.resetPinPointOdometry();
            }

            // ---------------- TELEMETRY ----------------
            if (tx != null) {
                telemetry.addData("Target Visible", true);
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
            } else {
                telemetry.addLine("No Target Detected - Turret Stopped");
            }

            telemetry.addData("Turret kP", turret.getkP());
            telemetry.addData("Turret kD", turret.getkD());
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
