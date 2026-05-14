package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

/**
 * TurretMechanism: Advanced "Infinite Rotation" Field-Oriented Version.
 *
 * FIXES APPLIED:
 *  1. kP/kD restored to effective defaults; TeleOp no longer overrides them down.
 *  2. D-term now differentiates turret encoder position (velocity), not raw error,
 *     eliminating derivative kick from tx noise.
 *  3. lastError and lastTurretPosDeg are now separate, properly-typed variables so
 *     mixing camera-space and encoder-space values is impossible.
 *  4. tx is low-pass filtered before use, eliminating camera noise micro-jitter.
 *  5. deltaTime is clamped on BOTH ends — no catastrophic D spikes from loop stalls.
 *  6. targetWorldAngleDeg is only re-pinned when the smoothed tx is stable enough
 *     (below a confidence threshold), preventing drift from noisy re-pinning.
 *  7. Feedforward sign comment clarified; sign constant kFF_SIGN lets you flip it
 *     in one place after empirical validation on the physical robot.
 */
public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;
    private Hardware hw;
    private MecanumCommand mecanumCommand;

    // --- PD Gains ---
    // Restored to effective values. Lower these only if the physical robot oscillates
    // after the D-term derivative-kick bug is fixed.
    private double kP = 0.022;
    private double kD = 0.0015;

    // --- Feedforward Gain ---
    // Compensates for robot rotation so the turret feels field-oriented.
    // kFF_SIGN: set to +1.0 or -1.0 after verifying on the robot.
    // If spinning the robot CW causes the turret to drift CW instead of staying put,
    // flip this sign.
    private double kFF      = 0.011;
    private double kFF_SIGN = -1.0;  // <-- flip to +1.0 if feedforward fights you

    // --- Low-pass filter alpha for tx ---
    // 0 = frozen (never updates), 1 = no filtering (raw).
    // 0.35 gives a good balance of lag vs. noise rejection.
    private static final double TX_FILTER_ALPHA = 0.35;
    private double smoothedTx = 0.0;

    // --- tx stability gate ---
    // Only re-pin the field target when smoothed tx is moving slowly.
    // If |smoothedTx - prevSmoothedTx| > this threshold the camera is still
    // swinging onto target; don't commit the pin yet.
    private static final double TX_STABILITY_THRESHOLD_DEG = 0.8;
    private double prevSmoothedTx = 0.0;

    // --- Field-centric memory ---
    private double targetWorldAngleDeg   = 0;
    private boolean targetFoundAtLeastOnce = false;

    // FIX: Track turret encoder position for the D-term, NOT error.
    // This avoids derivative kick entirely — we're measuring actual velocity.
    private double lastTurretPosDeg = 0;

    private double distanceTrack;

    private static final double ANGLE_TOLERANCE_DEG = 0.15;
    private static final double MAX_POWER           = 1.0;

    // deltaTime clamped: minimum 1 ms, maximum 50 ms.
    // The 50 ms cap prevents a stall/GC pause from producing a violent D spike.
    private static final double DT_MIN = 0.001;
    private static final double DT_MAX = 0.050;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private static final double LIMELIGHT_HEIGHT = 0.31;
    private static final double LIMELIGHT_ANGLE  = Math.toRadians(40);
    private static final double TARGET_HEIGHT     = 0.75;
    private static final double HOOD_MIN          = 0.36;
    private static final double HOOD_MAX          = 0.75;
    private static final double MIN_DISTANCE      = 0.3;
    private static final double MAX_DISTANCE      = 2.5;

    private static final double MIN_RPM = 3000;
    private static final double MAX_RPM = 4000;
    private double shootRPM = MIN_RPM;

    private boolean hasTarget = false;

    private static final double TICKS_PER_DEGREE = 1.8;

    // -------------------------------------------------------------------------

    public void init(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        turret = hw.llmotor;

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hood = hw.hood;

        lastTurretPosDeg = 0;
        loopTimer.reset();
    }

    public void setMecanumCommand(MecanumCommand mc) { this.mecanumCommand = mc; }
    public void setkP(double newkP)                  { this.kP = newkP; }
    public void setkD(double newkD)                  { this.kD = newkD; }
    public double getkP()                            { return kP; }
    public double getkD()                            { return kD; }
    public double getShootRPM()                      { return shootRPM; }
    public double getDistanceTrack()                 { return distanceTrack; }
    public boolean hasTarget()                       { return hasTarget; }

    // -------------------------------------------------------------------------

    /** Normalises angle to [-180, 180] so the turret always takes the shortest path. */
    private double wrapAngle(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // -------------------------------------------------------------------------

    public void update(Double tx, Double ty) {

        // --- 1. Safe deltaTime — clamped on both sides ---
        double deltaTime = Range.clip(loopTimer.seconds(), DT_MIN, DT_MAX);
        loopTimer.reset();

        // --- 2. Robot state from odometry ---
        double robotHeadingDeg   = 0;
        double robotRotVelocity  = 0;

        if (mecanumCommand != null) {
            robotHeadingDeg  = Math.toDegrees(mecanumCommand.getOdoHeading());
            robotRotVelocity = mecanumCommand.getHeadingVelocity(); // deg/s
        }

        // --- 3. Current turret world angle ---
        double currentTurretRelDeg = turret.getCurrentPosition() / TICKS_PER_DEGREE;
        double currentWorldAngle   = wrapAngle(robotHeadingDeg + currentTurretRelDeg);

        // --- 4. Turret encoder velocity for D-term (avoids derivative kick) ---
        // We differentiate the POSITION, not the error, so sudden tx changes
        // don't cause a power jolt.
        double turretPosDeg    = currentTurretRelDeg;
        double turretVelocity  = (turretPosDeg - lastTurretPosDeg) / deltaTime; // deg/s
        lastTurretPosDeg       = turretPosDeg;

        double error       = 0;
        double outputPower = 0;

        if (tx != null) {
            hasTarget = true;
            targetFoundAtLeastOnce = true;

            // --- 5. Low-pass filter on tx to kill camera noise ---
            smoothedTx = TX_FILTER_ALPHA * tx + (1.0 - TX_FILTER_ALPHA) * smoothedTx;

            // --- 6. Only re-pin the field target when tx has stabilised ---
            // If the camera is still swinging onto the goal the smoothed value
            // is changing quickly — wait until it settles before committing.
            double txDelta = Math.abs(smoothedTx - prevSmoothedTx);
            if (txDelta < TX_STABILITY_THRESHOLD_DEG) {
                targetWorldAngleDeg = wrapAngle(currentWorldAngle - smoothedTx);
            }
            prevSmoothedTx = smoothedTx;

            // Drive error from the filtered tx so noise doesn't feed the P term
            error = -smoothedTx;

        } else if (targetFoundAtLeastOnce) {
            // Target lost — hold the last known world-angle pin
            hasTarget  = false;
            smoothedTx = 0.0;       // reset filter so it doesn't have stale state
            // when the target reappears
            prevSmoothedTx = 0.0;
            error = wrapAngle(targetWorldAngleDeg - currentWorldAngle);
        }

        // --- 7. PD control ---
        // P term: proportional to angular error
        // D term: proportional to turret *velocity* (not error delta) — no kick
        double pTerm = error * kP;
        double dTerm = -turretVelocity * kD;   // negative: resist motion direction

        outputPower = (Math.abs(error) < ANGLE_TOLERANCE_DEG) ? 0.0 : (pTerm + dTerm);

        // --- 8. Feedforward: cancel robot rotation before error develops ---
        // kFF_SIGN must be validated on the physical robot.
        // If the turret drifts WITH robot spin, flip kFF_SIGN.
        double feedForward = kFF_SIGN * robotRotVelocity * kFF;

        double totalPower = Range.clip(outputPower + feedForward, -MAX_POWER, MAX_POWER);
        turret.setPower(totalPower);

        // --- 9. Hood angle and RPM from ty ---
        if (ty != null) {
            double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT)
                    / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));
            distanceTrack = distance;

            double clippedDist = Range.clip(distance * 0.9, MIN_DISTANCE, MAX_DISTANCE);
            double normalized  = (clippedDist - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);

            double hoodPos = HOOD_MIN + Math.pow(normalized, 3) * (HOOD_MAX - HOOD_MIN);

            if (distance > 0.55) hoodPos += 0.05;
            else                 hoodPos += 0.33;

            hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));
            shootRPM = Range.clip(
                    MIN_RPM + (normalized * 300) + (distance > 0.55 ? 270 : 0),
                    MIN_RPM, MAX_RPM);
        }
    }
}