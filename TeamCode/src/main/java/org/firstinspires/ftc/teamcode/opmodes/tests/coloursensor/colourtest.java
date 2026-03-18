package org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismTutorial;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;

import java.util.ArrayList;

@TeleOp(name = "ColourAuto2", group = "Competition")

public class colourtest extends LinearOpMode {

    // ---------------- SUBSYSTEMS -----------------
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private ShooterSubsystem shooterSubsystem;
    private TurretMechanismTutorial turret;

    // ---------------- LIMELIGHT ----------------
    private Limelight3A limelight;
    private LLResult llResult;

    // ---------------- HARDWARE ----------------
    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private Servo gate;

    // ---------------- COLOR SENSOR ----------------
    private ColorSensor colourSensor;

    private int red;
    private int green;
    private int blue;
    private int alpha;

    private boolean objectDetected = false;

    private ArrayList<String> artifacts = new ArrayList<>();

    private ElapsedTime detectionTimer = new ElapsedTime();

    // ---------------- TIMERS ----------------
    private final ElapsedTime pusherTimer = new ElapsedTime();

    // ---------------- STATE VARIABLES ----------------
    private double theta;

    private boolean autoAimEnabled = false;
    private boolean prevA = false;

    private boolean previousXState = false;
    private boolean previousYState = false;

    private boolean prevRightTrigger = false;
    private boolean prevLeftTrigger = false;

    private boolean togglePusher = false;
    private boolean isIntakeMotorOn = false;
    private boolean isOuttakeMotorOn = false;

    // ---------------- SORTER POSITIONS ----------------
    private static final double SLOT1 = 0.0;
    private static final double SLOT2 = 0.45;
    private static final double SLOT3 = 0.9;

    private final double GATE_UP = 1.0;
    private final double GATE_DOWN = 0.8;

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

        intake = hw.intake;
        shooter = hw.shooter;
        pusher = hw.pusher;
        gate = hw.gate;

        // Color sensor
        colourSensor = hardwareMap.get(ColorSensor.class, "colour");
        colourSensor.enableLed(true);

        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.sorter.setPosition(SLOT1);
        gate.setPosition(GATE_DOWN);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {

            // ---------------- DRIVE ----------------
            mecanumCommand.processOdometry();

            theta = mecanumCommand.normalMove(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            // ---------------- LIMELIGHT ----------------
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

            // ---------------- MANUAL TURRET ----------------
            double manualPower = 0;

            if (gamepad1.left_bumper) manualPower = 0.35;
            else if (gamepad1.right_bumper) manualPower = -0.35;

            if (manualPower != 0) {
                hw.llmotor.setPower(manualPower);
            }
            else if (autoAimEnabled) {
                turret.update(tx, ty);
            }
            else {
                hw.llmotor.setPower(0);
            }

            // ---------------- INTAKE TOGGLE ----------------
            boolean curRightTrigger = gamepad1.right_trigger > 0;

            if (curRightTrigger && !prevRightTrigger) {

                isIntakeMotorOn = !isIntakeMotorOn;

                intake.setPower(isIntakeMotorOn ? 0.8 : 0);

                gate.setPosition(isIntakeMotorOn ? GATE_UP : GATE_DOWN);
            }

            prevRightTrigger = curRightTrigger;

            // ---------------- OUTTAKE ----------------
            boolean curLeftTrigger = gamepad1.left_trigger > 0;

            if (curLeftTrigger && !prevLeftTrigger) {

                isOuttakeMotorOn = !isOuttakeMotorOn;

                intake.setPower(isOuttakeMotorOn ? -0.8 : 0);

                gate.setPosition(isOuttakeMotorOn ? GATE_UP : GATE_DOWN);
            }

            prevLeftTrigger = curLeftTrigger;

            // ---------------- SHOOTER ----------------
            boolean currentXState = gamepad1.x;

            if (currentXState && !previousXState) {

                isOuttakeMotorOn = !isOuttakeMotorOn;

                if (isOuttakeMotorOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    shooterSubsystem.spinup();
                }
                else {
                    shooterSubsystem.stopShooter();
                }
            }

            previousXState = currentXState;

            // ---------------- PUSHER ----------------
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

            // ---------------- COLOR SENSOR ----------------
            readSensor();

            detectColour();

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("----- LIMELIGHT -----");

            telemetry.addData("Target Visible", tx != null);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);

            telemetry.addLine("----- COLOR SENSOR -----");

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);

            telemetry.addData("Stored Objects", artifacts.size());

            telemetry.addLine("----- ODOMETRY -----");

            telemetry.addData("X", mecanumCommand.getX());
            telemetry.addData("Y", mecanumCommand.getY());
            telemetry.addData("Theta", mecanumCommand.getOdoHeading());

            telemetry.update();
        }
    }

    // ---------------- SENSOR READ ----------------

    private void readSensor() {

        red = colourSensor.red();
        green = colourSensor.green();
        blue = colourSensor.blue();
        alpha = colourSensor.alpha();
    }

    // ---------------- DETECT COLOUR ----------------

    private void detectColour() {

        if (artifacts.size() >= 3) return;

        if (detectionTimer.milliseconds() < 300) return;

        String detected = classifyColour();

        if (detected != null && !objectDetected) {

            objectDetected = true;

            artifacts.add(detected);

            moveSorter();

            detectionTimer.reset();
        }

        if (alpha < 40) {
            objectDetected = false;
        }
    }

    // ---------------- CLASSIFY ----------------

    private String classifyColour() {

        if (blue > green && blue > red && alpha > 60) {
            return "Purple";
        }

        if (green > blue + 20 && green > red && alpha > 60) {
            return "Green";
        }

        return null;
    }

    // ---------------- SORTER ----------------

    private void moveSorter() {

        switch (artifacts.size()) {

            case 1:
                hw.sorter.setPosition(SLOT1);
                break;

            case 2:
                hw.sorter.setPosition(SLOT2);
                break;

            case 3:
                hw.sorter.setPosition(SLOT3);
                break;
        }
    }
}
