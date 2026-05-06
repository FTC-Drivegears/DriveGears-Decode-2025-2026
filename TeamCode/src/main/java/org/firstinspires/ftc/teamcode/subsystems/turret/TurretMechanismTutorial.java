package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;
    private Hardware hw;
    private MecanumCommand mecanumCommand;

    // ---------------- TURRET PD CONTROL -----------------
    private double kP = 0.01;
    private double kD = 0.005;

    private double distanceTrack;
    private double lastError = 0;
    private final double ANGLE_TOLERANCE_DEG = 0.5;
    private final double MAX_POWER = 0.6;
    private final double HOLD_POWER = 0.5;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- HOOD / SHOOTER ----------------
    private final double LIMELIGHT_HEIGHT = 0.31;
    private final double LIMELIGHT_ANGLE = Math.toRadians(40);
    private final double TARGET_HEIGHT = 0.75;
    private final double HOOD_MIN = 0.36;
    private final double HOOD_MAX = 0.75;
    private final double MIN_DISTANCE = 0.3;
    private final double MAX_DISTANCE = 2.0;

    private final double MIN_RPM = 3000;
    private final double MAX_RPM = 4000;
    private double shootRPM = MIN_RPM;

    // ---------------- TARGET MEMORY ----------------
    private boolean hasTarget = false;
    private boolean targetLostInitialized = false;

    // last heading for delta calculation
    private double lastHeadingDeg = 0;

    // accumulated fractional ticks to prevent truncation error
    private double accumulatedTickError = 0;

    // Turret ticks per degree — measured: 162 ticks per 90 degrees = 1.8 ticks/degree
    private final double TICKS_PER_DEGREE = 1.8;

    // ---------------- DEBUG ----------------
    private double debugHeadingDelta = 0;
    private int debugTargetTicks = 0;

    public void init(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        turret = hw.llmotor;
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hood = hw.hood;
        loopTimer.reset();
    }

    public void setMecanumCommand(MecanumCommand mc) { this.mecanumCommand = mc; }
    public void resetTimer() { loopTimer.reset(); }
    public void setkP(double newkP) { kP = newkP; }
    public void setkD(double newkD) { kD = newkD; }
    public double getkP() { return kP; }
    public double getkD() { return kD; }
    public double getShootRPM() { return shootRPM; }
    public double getDistanceTrack() { return distanceTrack; }
    public boolean hasTarget() { return hasTarget; }
    public double getDebugHeadingDelta() { return debugHeadingDelta; }
    public int getDebugTargetTicks() { return debugTargetTicks; }

    public void update(Double tx, Double ty) {
        double deltaTime = loopTimer.seconds();
        deltaTime = Math.max(deltaTime, 0.01);
        loopTimer.reset();

        double currentHeadingRad = (mecanumCommand != null)
                ? mecanumCommand.getOdoHeading() : 0;
        double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

        if (tx != null) {
            // ---------------- LIMELIGHT VISIBLE - PD control ----------------
            if (turret.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            double error = -tx;
            double dTerm = ((error - lastError) / deltaTime) * kD;
            double power = (Math.abs(error) < ANGLE_TOLERANCE_DEG)
                    ? 0 : Range.clip(error * kP + dTerm, -MAX_POWER, MAX_POWER);

            turret.setPower(power);
            lastError = error;
            hasTarget = true;
            targetLostInitialized = false;
            accumulatedTickError = 0;
            lastHeadingDeg = currentHeadingDeg;

        } else if (hasTarget) {
            // ---------------- TARGET LOST - pure heading delta tracking ----------------
            if (!targetLostInitialized) {
                lastHeadingDeg = currentHeadingDeg;
                targetLostInitialized = true;
                accumulatedTickError = 0;
                turret.setTargetPosition(turret.getCurrentPosition());
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turret.setPower(HOLD_POWER);
            }

            // how much robot rotated this frame
            double headingDeltaDeg = currentHeadingDeg - lastHeadingDeg;
            while (headingDeltaDeg > 180) headingDeltaDeg -= 360;
            while (headingDeltaDeg < -180) headingDeltaDeg += 360;

            // accumulate fractional ticks to prevent truncation error
            accumulatedTickError += headingDeltaDeg * TICKS_PER_DEGREE;
            int ticksDelta = (int) accumulatedTickError;
            accumulatedTickError -= ticksDelta;

            // shift target opposite to robot rotation
            int newTarget = turret.getTargetPosition() - ticksDelta;

            debugHeadingDelta = headingDeltaDeg;
            debugTargetTicks = newTarget;

            turret.setTargetPosition(newTarget);
            turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            turret.setPower(HOLD_POWER);

            lastHeadingDeg = currentHeadingDeg;
            lastError = 0;

        } else {
            // ---------------- NO TARGET EVER SEEN - brake hold ----------------
            if (turret.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
            turret.setPower(0);
            lastError = 0;
        }

        // ---------------- HOOD + RPM ----------------
        if (ty != null) {
            double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT)
                    / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));

            distanceTrack = distance;
            distance *= 0.9;
            distance = Range.clip(distance, MIN_DISTANCE, MAX_DISTANCE);

            double normalized = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
            normalized = Range.clip(normalized, 0, 1);

            double hoodCurve = Math.pow(normalized, 3.0);
            double hoodPos = HOOD_MIN + hoodCurve * (HOOD_MAX - HOOD_MIN);

            if (distance > 0.55) hoodPos += 0.05;
            if (distance < 0.55) hoodPos += 0.412;
            if (distance < 0.55) hoodPos += 0.25;

            hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));

            shootRPM = MIN_RPM + normalized * 300;
            if (distance > 0.55) shootRPM += 270;
            shootRPM = Range.clip(shootRPM, MIN_RPM, MAX_RPM);
        }
    }
}