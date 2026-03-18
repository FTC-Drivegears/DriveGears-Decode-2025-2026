package org.firstinspires.ftc.teamcode.opmodes.competition;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
import org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor.ColourSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismTutorial;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

import java.util.ArrayList;

@TeleOp(name = "TurretAutoAlignOpModeTutorial", group = "TeleOp")
public class TurretAutoAlignOpModeTutorial extends LinearOpMode {

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
    private DcMotor shooter;
    private Servo pusher;
    private Servo gate;

    private Servo light;

    private double theta;
    private double sorterPosition = 0.0;

    private final double GATE_UP = 0.689;
    private final double GATE_DOWN = 0.6;

    // ---------------- TIMERS ----------------
    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ---------------- INITIALIZATION ----------------
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);
        turret.setkP(0.035);
        turret.setkD(0.001);

        limelight = hw.limelight;
        limelight.pipelineSwitch(8);
        limelight.start();

        colourSubsystem = new ColourSensorSubsystem(hardwareMap, hw);

        intake = hw.intake;
        shooter = hw.shooter;
        pusher = hw.pusher;
        gate = hw.gate;
        light = hw.light;

        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.sorter.setPosition(0.0);
        hw.light.setPosition(0.0);

        gate.setPosition(GATE_DOWN);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        boolean autoAimEnabled = false;
        boolean prevA = false;

        if (sorterSubsystem == null) {
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");
        }

        waitForStart();

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

            // ---------------- LIMELIGHT DATA ----------------
            llResult = limelight.getLatestResult();

            Double tx = null;
            Double ty = null;

            if (llResult != null && llResult.isValid()) {
                tx = llResult.getTx();
                ty = llResult.getTy();
            }

// ---------------- AUTO AIM TOGGLE ----------------
            boolean curA = gamepad1.a;

            if (curA && !prevA) {
                autoAimEnabled = !autoAimEnabled;
            }

            prevA = curA;

// ---------------- MANUAL OVERRIDE ----------------
            double manualPower = 0;

            if (gamepad1.left_bumper) {
                manualPower = 0.35;
            }
            else if (gamepad1.right_bumper) {
                manualPower = -0.35;
            }

// ---------------- TURRET CONTROL ----------------
            if (manualPower != 0) {

                // manual override
                hw.llmotor.setPower(manualPower);

            }
            else if (autoAimEnabled) {

                // auto aim using odometry + limelight
                turret.update(tx, ty);

            }
            else {

                // idle
                hw.llmotor.setPower(0);

            }
            if (autoAimEnabled && tx != null) {
                RobotLog.i(String.format(
                        "time:%.2f target:%.1f position:%.1f error:%.1f power:%.2f",
                        getRuntime(),
                        0.0,
                        tx,
                        -tx,
                        hw.llmotor.getPower()
                ));
            }

            // ---------------- INTAKE TOGGLE ----------------
            boolean curRightTrigger = gamepad1.right_trigger > 0;
            if (curRightTrigger && !prevRightTrigger) {
                isIntakeMotorOn = !isIntakeMotorOn;
                intake.setPower(isIntakeMotorOn ? 0.8 : 0);
                gate.setPosition(isIntakeMotorOn ? GATE_UP : GATE_DOWN);
            }
            prevRightTrigger = curRightTrigger;
            colourSubsystem.update(isIntakeMotorOn);


            // ---------------- OUTTAKE TOGGLE -----------------
            boolean curLeftTrigger = gamepad1.left_trigger > 0;
            if (curLeftTrigger && !prevLeftTrigger) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
                intake.setPower(isOuttakeMotorOn ? -0.8 : 0);
                gate.setPosition(isOuttakeMotorOn ? GATE_UP : GATE_DOWN);
            }
            prevLeftTrigger = curLeftTrigger;

//             ---------------- SHOOTER TOGGLE ----------------
            boolean currentXState = gamepad1.x;

            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
                if (isOuttakeMotorOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    shooterSubsystem.spinup();
                } else {
                    shooterSubsystem.stopShooter();
                    light.setPosition(0.0);
                }
            }
            previousXState = currentXState;

            if (isOuttakeMotorOn) {
                if (shooterSubsystem.isRPMReached()) { //add && tx<5
                    light.setPosition(0.3);
                } else {
                    light.setPosition(0.0);
                }
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
            telemetry.addData("Target Visible", tx != null);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Turret kP", turret.getkP());
            telemetry.addData("Turret kD", turret.getkD());
            telemetry.addData("Shooter RPM", turret.getShootRPM());
            telemetry.addData("Intake On", isIntakeMotorOn);
            telemetry.addData("Outtake On", isOuttakeMotorOn);
            telemetry.addData("Turret Ticks", hw.llmotor.getCurrentPosition());

            telemetry.addLine("---------------------------------");
            telemetry.addData("Robot X", mecanumCommand.getX());
            telemetry.addData("Robot Y", mecanumCommand.getY());
            telemetry.addData("Theta (rad)", mecanumCommand.getOdoHeading());
            telemetry.addData("Auto Aim Enabled", autoAimEnabled);
            telemetry.addData("Manual Override", gamepad1.right_bumper || gamepad1.left_bumper);

            telemetry.addData("Red", colourSubsystem.getRed());
            telemetry.addData("Green", colourSubsystem.getGreen());
            telemetry.addData("Blue", colourSubsystem.getBlue());
            telemetry.addData("Alpha", colourSubsystem.getAlpha());
            telemetry.addData("Red2", colourSubsystem.getRed2());
            telemetry.addData("Green2", colourSubsystem.getGreen2());
            telemetry.addData("Blue2", colourSubsystem.getBlue2());
            telemetry.addData("Alpha2", colourSubsystem.getAlpha2());
            telemetry.addData("Detected Count", colourSubsystem.getCount());
            telemetry.update();
        }
    }
}
