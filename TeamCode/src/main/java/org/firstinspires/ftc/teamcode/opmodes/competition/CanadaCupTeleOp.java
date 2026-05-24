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
    private Servo pusher_R;
    private Servo pusher_L;
    private Servo gate;

    private Servo leftLight;
    private Servo rightLight;
    private double theta;
    private double sorterPosition = 0.0;

    // ---------------- SOFT LIMITS ----------------
    private static final int TURRET_MIN_TICKS = -240;
    private static final int TURRET_MAX_TICKS =  240;

    // ---------------- TIMERS ----------------
    private final ElapsedTime sorterTimer = new ElapsedTime();
    private final ElapsedTime pusherTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ---------------- INITIALIZATION ----------------
        Hardware.resetInstance(); // Force fresh hardware init — avoids stale singleton from auto
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);

        turret = new TurretMechanismTutorial();
        turret.init(hardwareMap);
        turret.setMecanumCommand(mecanumCommand);
        turret.setkP(0.01);
        turret.setkD(0.001);

        limelight = hw.limelight;
        limelight.pipelineSwitch(8);
        limelight.start();

        sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "pgg");

        colourSubsystem = new ColourSensorSubsystem(hardwareMap, hw, sorterSubsystem);

        intake = hw.intake;
        shooter = hw.shooter;
        pusher_R = hw.pusher_R;
        pusher_L = hw.pusher_L;
        leftLight = hw.leftLight;
        rightLight = hw.rightLight;
        gate = hw.gate;

        pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
        pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
        hw.sorter.setPosition(0.0);
        hw.leftLight.setPosition(0.0);
        hw.hood.setPosition(0.36); //min
        hw.rightLight.setPosition(1.0);
        gate.setPosition(0.6);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.llmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // NOTE: Pinpoint odometry heading is intentionally NOT reset here.
        // Heading carries over from auto so field-centric drive is correct
        // from the first driver input. Use gamepad1.start to re-zero mid-match if needed.

        boolean autoAimEnabled = false;
        boolean prevA = false;

        waitForStart();

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
        while (opModeIsActive()) {
            limelight.pipelineSwitch(8);

            // ---------------- DRIVE (FIELD-CENTRIC) ----------------
            mecanumCommand.processOdometry();

            double heading = mecanumCommand.getOdoHeading(); // Pinpoint returns radians, CW = negative

            // Raw driver inputs
            double inputY = -gamepad1.left_stick_y; // forward/back
            double inputX =  gamepad1.left_stick_x; // strafe
            double inputR =  gamepad1.right_stick_x; // rotation stays robot-centric

            // Rotate the translation vector by -heading to align to field forward
            double fieldX = inputX * Math.cos(-heading) - inputY * Math.sin(-heading);
            double fieldY = inputX * Math.sin(-heading) + inputY * Math.cos(-heading);

            // NOTE: Field-centric rotation already applied above via cos/sin math.
            // Use normalMove here, NOT fieldOrientedMove — that would double-rotate.
            theta = mecanumCommand.normalMove(fieldY, fieldX, inputR);

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

                if (autoAimEnabled) {
                    leftLight.setPosition(0.44);
                } else {
                    leftLight.setPosition(0.0);
                }

            }
            prevA = curA;

            // ---------------- MANUAL OVERRIDE ----------------
            double manualPower = 0;
            int turretPos = hw.llmotor.getCurrentPosition();

            boolean curLeftBumper = gamepad1.left_bumper;
            boolean curRightBumper = gamepad1.right_bumper;

            if (curLeftBumper) {
                if (turretPos >= TURRET_MAX_TICKS) {
                    manualPower = -0.3;
                } else {
                    double distToLimit = TURRET_MAX_TICKS - turretPos;
                    double scale = Math.min(distToLimit / 50.0, 1.0);
                    manualPower = 0.15 + 0.2 * scale;
                }
            } else if (curRightBumper) {
                if (turretPos <= TURRET_MIN_TICKS) {
                    manualPower = 0.3;
                } else {
                    double distToLimit = turretPos - TURRET_MIN_TICKS;
                    double scale = Math.min(distToLimit / 50.0, 1.0);
                    manualPower = -(0.15 + 0.2 * scale);
                }
            }

            // Detect manual -> auto transition and reset turret control state
            boolean isManual = (manualPower != 0);
            if (prevManual && !isManual) {
            }
            prevManual = isManual;

            // ---------------- TURRET CONTROL ----------------
            if (turretPos > TURRET_MAX_TICKS) {
                hw.llmotor.setPower(-0.3);
            } else if (turretPos < TURRET_MIN_TICKS) {
                hw.llmotor.setPower(0.3);
            } else if (manualPower != 0) {
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

            if (sorterSubsystem.getArtifactCount() == 3) {
                attemptedFullIntake = true;
            } else {
                attemptedFullIntake = false;
            }

            if (curRightTrigger && !prevRightTrigger) {
                isIntakeMotorOn = !isIntakeMotorOn;

                if (isIntakeMotorOn) {
                    isOuttakeMotorOn = false;
                    intake.setPower(0.8);
                } else {
                    intake.setPower(0);
                }
            }
            prevRightTrigger = curRightTrigger;

            if (sorterSubsystem.getArtifactCount() == 3 && isIntakeMotorOn && !attemptedFullIntake) {
                isIntakeMotorOn = false;
                intake.setPower(0);
            }

            colourSubsystem.update(isIntakeMotorOn);

            // ---------------- OUTTAKE TOGGLE ----------------
            boolean curLeftTrigger = gamepad1.left_trigger > 0;
            if (curLeftTrigger && !prevLeftTrigger) {
                isOuttakeMotorOn = !isOuttakeMotorOn;

                if (isOuttakeMotorOn) {
                    isIntakeMotorOn = false;
                    intake.setPower(-0.8);
                } else {
                    intake.setPower(0);
                }
            }
            prevLeftTrigger = curLeftTrigger;

            // ---------------- GATE CONTROL ----------------
            if (isIntakeMotorOn || isOuttakeMotorOn) {
                gate.setPosition(0.7);
            } else {
                gate.setPosition(0.6);
            }

            // ---------------- SHOOTER TOGGLE ----------------
            boolean currentXState = gamepad1.x;

            if (currentXState && !previousXState) {
                isShooterOn = !isShooterOn;
                if (isShooterOn) {
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    shooterSubsystem.spinup();
                } else {
                    shooterSubsystem.stopShooter();
                    leftLight.setPosition(0.0);
                }
            }
            previousXState = currentXState;

            if (isShooterOn && tx != null && Math.abs(tx) < 3) {
                if (shooterSubsystem.isRPMReached()) {
                    leftLight.setPosition(0.3);
                } else {
                    leftLight.setPosition(0.0);
                }
            }

            // ---------------- PUSHER CONTROL ----------------
            boolean currentYState = gamepad1.y;
            if (currentYState && !previousYState) {
                if (!togglePusher) {
                    pusher_R.setPosition(PusherConsts.PUSHER_UP_POSITION_R);
                    pusher_L.setPosition(PusherConsts.PUSHER_UP_POSITION_L);
                    pusherTimer.reset();
                    togglePusher = true;
                }
            }
            previousYState = currentYState;

            if (togglePusher && pusherTimer.milliseconds() >= 500) {
                pusher_R.setPosition(PusherConsts.PUSHER_DOWN_POSITION_R);
                pusher_L.setPosition(PusherConsts.PUSHER_DOWN_POSITION_L);
                togglePusher = false;
            }

            // ---------------- SORTER ----------------
            if (gamepad1.b && sorterTimer.milliseconds() > 500) {
//                sorterPosition = (sorterPosition + 1) % 3;
//                sorterTimer.reset();
//                if (sorterPosition == 0.0) hw.sorter.setPosition(0.0);
//                else if (sorterPosition == 1) hw.sorter.setPosition(0.43);
//                else hw.sorter.setPosition(0.875);
                sorterTimer.reset();
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
            }

            if (gamepad1.dpad_left && !prevDpadLeft) sorterSubsystem.startQuickfire();
            if (sorterSubsystem.isActive())          sorterSubsystem.quickfireState();
            prevDpadLeft = gamepad1.dpad_left;

            if (gamepad1.dpad_right) sorterSubsystem.stopQuickfire();

            // ---------------- ODOMETRY RESET ----------------
            // Resets heading to 0 (current robot direction becomes new field-forward).
            // Useful mid-match if driver gets disoriented.
            if (gamepad1.start) {
                mecanumCommand.resetPinPointOdometry();
            }

            // ---------------- TELEMETRY ----------------
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
            telemetry.addData("Last Red", colourSubsystem.getLastValues()[0]);
            telemetry.addData("Last Green", colourSubsystem.getLastValues()[1]);
            telemetry.addData("Last Blue", colourSubsystem.getLastValues()[2]);
            telemetry.addData("Last Alpha", colourSubsystem.getLastValues()[3]);
            telemetry.addData("Detected Count", sorterSubsystem.getArtifactCount());
            telemetry.addData("Current Balls", Arrays.toString(sorterSubsystem.getSorterList()));
            telemetry.addData("Current Sorter Position", sorterSubsystem.getSorterPos());
            telemetry.addData("Selected Colour", sorterSubsystem.selectedColour);
            telemetry.update();
        }
    }
}