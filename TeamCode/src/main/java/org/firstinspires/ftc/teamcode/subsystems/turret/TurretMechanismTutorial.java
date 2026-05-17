package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * TurretMechanism v10.1 — True Negative Feedback Alignment.
 *
 * KEY FIXES:
 * 1. Corrected control loop sign reversal: Limelight tx is negative to the left.
 * To turn left toward the target, the turret needs positive motor power.
 * Therefore, error is now correctly defined as -smoothedTx.
 * 2. Corrected targetWorldAngleDeg calculation: Targets to the right (positive tx)
 * exist at a smaller world angle. Changed to: currentWorldAngleDeg - smoothedTx.
 * 3. Tightened MIN_POWER_FADE_WINDOW_DEG to 0.5 degrees to provide strong corrective
 * torque close to the center without causing stalling or steady-state offsets.
 */
public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;
    private Hardware hw;
    private MecanumCommand mecanumCommand;

    // --- PID gains ---
    private double kP = 0.04;
    private double kI = 0.015;
    private double kD = 0.002;

    private static final double MAX_INTEGRAL = 0.25;
    private double integralSum = 0.0;
    private double prevError   = 0.0;

    private static final double MIN_MOTOR_POWER = 0.15;
    private static final double MAX_OUTPUT_POWER = 0.75;

    // Window where minimum power scales down to 0 to prevent close-range oscillation
    private static final double MIN_POWER_FADE_WINDOW_DEG = 0.5;

    // Limelight smoothing
    private static final double TX_FILTER_ALPHA            = 0.55;
    private static final double TX_STABILITY_THRESHOLD_DEG = 0.8;
    private double smoothedTx     = 0.0;
    private double prevSmoothedTx = 0.0;

    private int stableFrameCount = 0;
    private static final int STABLE_FRAMES_REQUIRED = 2;
    private int consecutiveTargetFrames = 0;
    private static final int TARGET_FRAMES_REQUIRED = 3;

    private double lastRawTx = 0;
    private int staleTxCount = 0;
    private static final double STALE_TX_THRESHOLD = 0.05;
    private static final int STALE_TX_MAX_FRAMES   = 6;

    private double  targetWorldAngleDeg    = 0;
    private boolean targetFoundAtLeastOnce = false;
    private boolean worldAngleConfident    = false;

    // Soft confidence decay
    private int blindFrameCount = 0;
    private static final int BLIND_FULL_POWER_FRAMES = 15;
    private static final int BLIND_RAMP_END_FRAMES   = 40;
    private static final double BLIND_MIN_SCALE       = 0.3;
    private static final int BLIND_CONFIDENCE_EXPIRE  = 60;

    // D-term suppression on reacquisition
    private int framesSinceAcquisition = 999;
    private static final int DTERM_SUPPRESS_FRAMES = 2;

    // World velocity low-pass filter
    private double filteredWorldVelocity = 0.0;
    private static final double VELOCITY_FILTER_ALPHA = 0.5;

    private double lastTurretPosDeg  = 0;
    private double prevWorldAngleDeg = 0;
    private boolean firstUpdate      = true;
    private double distanceTrack     = 0;

    private static final double MAX_EXPECTED_TX_DRIFT_DEG = 15.0;

    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.050;

    private final ElapsedTime loopTimer  = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

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

    private BufferedWriter logWriter   = null;
    private boolean        loggingEnabled = false;

    public void init(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        turret  = hw.llmotor;

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hood                    = hw.hood;
        lastTurretPosDeg        = 0;
        prevWorldAngleDeg       = 0;
        firstUpdate             = true;
        targetFoundAtLeastOnce  = false;
        worldAngleConfident     = false;
        integralSum             = 0;
        prevError               = 0;
        stableFrameCount        = 0;
        consecutiveTargetFrames = 0;
        staleTxCount            = 0;
        lastRawTx               = 0;
        blindFrameCount         = 0;
        framesSinceAcquisition  = 999;
        filteredWorldVelocity   = 0;
        loopTimer.reset();
        totalTimer.reset();

        try {
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String path      = "/sdcard/FIRST/turret_log_" + timestamp + ".csv";
            logWriter        = new BufferedWriter(new FileWriter(path));
            loggingEnabled   = true;
            logWriter.write("time_ms,raw_tx,smoothed_tx,error,turret_rel_deg,world_angle_deg," +
                    "robot_heading_deg,world_velocity,filtered_velocity,p_term,i_term,d_term,output_power," +
                    "motor_power,has_target,target_world_angle_deg,tx_rejected,stale_tx_count," +
                    "consec_target_frames,blind_frames,confident,blind_scale\n");
            logWriter.flush();
        } catch (IOException e) {
            loggingEnabled = false;
        }
    }

    public void closeLog() {
        if (logWriter != null) {
            try {
                logWriter.flush();
                logWriter.close();
            } catch (IOException e) {}
            logWriter = null;
        }
    }

    public void setMecanumCommand(MecanumCommand mc) { this.mecanumCommand = mc; }
    public void setkP(double v)  { this.kP = v; }
    public void setkI(double v)  { this.kI = v; }
    public void setkD(double v)  { this.kD = v; }

    public double getkP()            { return kP; }
    public double getkI()            { return kI; }
    public double getkD()            { return kD; }
    public double getShootRPM()      { return shootRPM; }
    public double getDistanceTrack() { return distanceTrack; }
    public boolean hasTarget()       { return hasTarget; }

    private double wrapAngle(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double getBlindPowerScale() {
        if (blindFrameCount <= BLIND_FULL_POWER_FRAMES) {
            return 1.0;
        } else if (blindFrameCount <= BLIND_RAMP_END_FRAMES) {
            double progress = (double)(blindFrameCount - BLIND_FULL_POWER_FRAMES)
                    / (BLIND_RAMP_END_FRAMES - BLIND_FULL_POWER_FRAMES);
            return 1.0 - progress * (1.0 - BLIND_MIN_SCALE);
        } else {
            return BLIND_MIN_SCALE;
        }
    }

    public void update(Double tx, Double ty) {
        double deltaTime = loopTimer.seconds();
        loopTimer.reset();

        if (Double.isNaN(deltaTime) || deltaTime < DT_MIN) {
            deltaTime = DT_MIN;
        } else if (deltaTime > DT_MAX) {
            deltaTime = DT_MAX;
        }

        double robotHeadingDeg = (mecanumCommand != null) ? Math.toDegrees(mecanumCommand.getOdoHeading()) : 0;

        if (Double.isNaN(robotHeadingDeg)) {
            robotHeadingDeg = 0;
        }

        double currentTurretRelDeg  = turret.getCurrentPosition() / TICKS_PER_DEGREE;
        double currentWorldAngleDeg = wrapAngle(robotHeadingDeg + currentTurretRelDeg);

        if (firstUpdate) {
            prevWorldAngleDeg = currentWorldAngleDeg;
            firstUpdate = false;
        }

        double rawWorldVelocity = wrapAngle(currentWorldAngleDeg - prevWorldAngleDeg) / deltaTime;
        prevWorldAngleDeg       = currentWorldAngleDeg;
        lastTurretPosDeg        = currentTurretRelDeg;

        if (Double.isFinite(rawWorldVelocity)) {
            filteredWorldVelocity = VELOCITY_FILTER_ALPHA * rawWorldVelocity
                    + (1.0 - VELOCITY_FILTER_ALPHA) * filteredWorldVelocity;
        }

        double error       = 0;
        double outputPower = 0;
        double rawTx       = (tx != null) ? tx : 0.0;
        double blindScale  = 1.0;

        // Camera-space equivalent of where the target should be based on our pin
        double expectedTx = wrapAngle(currentWorldAngleDeg - targetWorldAngleDeg);

        boolean txRejected = false;
        if (tx != null && worldAngleConfident) {
            double txError = Math.abs(wrapAngle(tx - expectedTx));
            if (txError > MAX_EXPECTED_TX_DRIFT_DEG) {
                txRejected = true;
            }
        }

        boolean txIsStale = false;
        if (tx != null && !txRejected) {
            if (Math.abs(tx - lastRawTx) < STALE_TX_THRESHOLD && Math.abs(filteredWorldVelocity) > 20.0) {
                staleTxCount++;
            } else {
                staleTxCount = 0;
            }
            lastRawTx = tx;
            txIsStale = (staleTxCount >= STALE_TX_MAX_FRAMES);
        }

        if (tx != null && !txRejected && !txIsStale && Math.abs(tx) < 25.0) {
            targetFoundAtLeastOnce = true;
            consecutiveTargetFrames++;
            blindFrameCount = 0;

            if (!hasTarget) {
                smoothedTx       = tx;
                prevSmoothedTx   = tx;
                integralSum      = 0;
                stableFrameCount = 0;
                framesSinceAcquisition = 0;
            } else {
                smoothedTx = TX_FILTER_ALPHA * tx + (1.0 - TX_FILTER_ALPHA) * smoothedTx;
                framesSinceAcquisition++;
            }
            hasTarget = true;

            double txDelta = Math.abs(smoothedTx - prevSmoothedTx);
            prevSmoothedTx = smoothedTx;

            if (worldAngleConfident) {
                // Corrected sign: Targets to the right (positive tx) are at a smaller world angle
                targetWorldAngleDeg = wrapAngle(currentWorldAngleDeg - smoothedTx);
            } else {
                if (txDelta < TX_STABILITY_THRESHOLD_DEG) {
                    stableFrameCount++;
                    if (stableFrameCount >= STABLE_FRAMES_REQUIRED && consecutiveTargetFrames >= TARGET_FRAMES_REQUIRED) {
                        targetWorldAngleDeg = wrapAngle(currentWorldAngleDeg - smoothedTx);
                        worldAngleConfident = true;
                    }
                } else {
                    stableFrameCount = 0;
                }
            }

            // Corrected sign: Negative tx (left) requires positive power to turn left
            error = -smoothedTx;

        } else if (targetFoundAtLeastOnce) {
            hasTarget               = false;
            smoothedTx              = 0.0;
            prevSmoothedTx          = 0.0;
            stableFrameCount        = 0;
            consecutiveTargetFrames = 0;
            framesSinceAcquisition  = 999;

            blindFrameCount++;

            if (worldAngleConfident) {
                if (blindFrameCount >= BLIND_CONFIDENCE_EXPIRE) {
                    worldAngleConfident = false;
                    integralSum = 0;
                    error = 0;
                    blindScale = 0;
                } else {
                    error = -expectedTx;
                    blindScale = getBlindPowerScale();
                }
            } else {
                error = 0;
                blindScale = 0;
            }
        } else {
            hasTarget               = false;
            consecutiveTargetFrames = 0;
            blindScale = 0;
        }

        // --- PID Controller ---
        double pTerm = 0, iTerm = 0, dTerm = 0;

        pTerm = kP * error;

        if (error * prevError < 0) integralSum = 0;
        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -MAX_INTEGRAL / kI, MAX_INTEGRAL / kI);
        iTerm = integralSum * kI;

        if (framesSinceAcquisition >= DTERM_SUPPRESS_FRAMES) {
            dTerm = -filteredWorldVelocity * kD;
        }

        outputPower = pTerm + iTerm + dTerm;

        if (!hasTarget && blindScale < 1.0) {
            outputPower *= blindScale;
        }

        // Overcome static friction without letting the power clip to a flat minimum far away
        double absError = Math.abs(error);
        if (absError > 0.05 && Math.abs(outputPower) < MIN_MOTOR_POWER) {
            double minPowerScale = 1.0;
            if (absError < MIN_POWER_FADE_WINDOW_DEG) {
                minPowerScale = absError / MIN_POWER_FADE_WINDOW_DEG;
            }
            outputPower = Math.copySign(MIN_MOTOR_POWER * minPowerScale, outputPower);
        }

        prevError = error;

        if (Double.isNaN(outputPower)) {
            outputPower = 0;
        }

        double totalPower = Range.clip(outputPower, -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);
        turret.setPower(totalPower);

        // Logging
        if (loggingEnabled && logWriter != null) {
            try {
                logWriter.write(
                        totalTimer.milliseconds() + "," +
                                rawTx + "," +
                                smoothedTx + "," +
                                error + "," +
                                currentTurretRelDeg + "," +
                                currentWorldAngleDeg + "," +
                                robotHeadingDeg + "," +
                                rawWorldVelocity + "," +
                                filteredWorldVelocity + "," +
                                pTerm + "," +
                                iTerm + "," +
                                dTerm + "," +
                                outputPower + "," +
                                totalPower + "," +
                                (hasTarget ? 1 : 0) + "," +
                                targetWorldAngleDeg + "," +
                                (txRejected ? 1 : 0) + "," +
                                staleTxCount + "," +
                                consecutiveTargetFrames + "," +
                                blindFrameCount + "," +
                                (worldAngleConfident ? 1 : 0) + "," +
                                blindScale + "\n"
                );
                if ((int)(totalTimer.milliseconds()) % 1000 < 20) logWriter.flush();
            } catch (IOException e) { loggingEnabled = false; }
        }

        // Hood and RPM
        if (ty != null) {
            double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));
            distanceTrack = distance;
            double clippedDist = Range.clip(distance * 0.9, MIN_DISTANCE, MAX_DISTANCE);
            double normalized  = (clippedDist - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
            double hoodPos = HOOD_MIN + Math.pow(normalized, 3) * (HOOD_MAX - HOOD_MIN);
            hoodPos += (distance > 0.55) ? 0.05 : 0.33;
            hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));
            shootRPM = Range.clip(MIN_RPM + (normalized * 300) + (distance > 0.55 ? 270 : 0), MIN_RPM, MAX_RPM);
        }
    }
}