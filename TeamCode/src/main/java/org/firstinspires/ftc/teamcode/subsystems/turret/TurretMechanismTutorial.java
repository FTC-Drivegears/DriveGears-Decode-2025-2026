package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * TurretMechanism v17.3 — Added stuck-motor / integral windup protection.
 *
 * v17.2 fixes retained unchanged.
 *
 * New in v17.3:
 * STUCK DETECTION: If significant error exists but the turret hasn't moved
 * STUCK_MOVEMENT_DEG within STUCK_FRAME_THRESHOLD consecutive frames, the
 * integral is flushed. Prevents the integral winding up to max during an
 * Expansion Hub disconnect and slamming the turret on reconnect.
 */
public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;
    private Hardware hw;
    private MecanumCommand mecanumCommand;

    // --- PID gains ---
    private double kP = 0.036;
    private double kI = 0.012;
    private double kD = 0; //0.02;

    private static final double MAX_INTEGRAL            = 0.10;
    private static final double INTEGRAL_SEPARATION_DEG = 3.0;
    private double integralSum = 0.0;
    private double prevError   = 0.0;

    private static final double FEEDFORWARD_STICTION_POWER = 0.05; //0.12;
    private static final double MAX_OUTPUT_POWER            = 0.80;
    private static final double ERROR_DEADBAND_DEG          = 1.0;
    private static final double MIN_POWER_FADE_WINDOW_DEG   = 0.8;

    // --- Stuck-motor / integral windup protection ---
    private double lastCheckedPosDeg              = 0;
    private int    stuckFrameCount                = 0;
    private static final int    STUCK_FRAME_THRESHOLD = 20;    // ~400 ms at 20 ms/loop
    private static final double STUCK_MOVEMENT_DEG    = 0.5;   // deg — less than this = not moving
    private static final double STUCK_ERROR_THRESHOLD = 3.0;   // only check when error is meaningful

    // Manual mode
    private boolean manualMode  = false;
    private double  manualPower = 0.0;

    // Tx filter
    private static final double MIN_ALPHA                  = 0.15;
    private static final double MAX_ALPHA                  = 0.50;
    private static final double TX_STABILITY_THRESHOLD_DEG = 0.8;
    private double smoothedTx     = 0.0;
    private double prevSmoothedTx = 0.0;

    private int stableFrameCount        = 0;
    private static final int STABLE_FRAMES_REQUIRED = 1;
    private int consecutiveTargetFrames = 0;
    private static final int TARGET_FRAMES_REQUIRED = 1;

    private static final int PIN_UPDATE_MIN_FRAMES = 3;

    // Stale-tx detection
    private double lastRawTx    = 0;
    private int    staleTxCount = 0;
    private static final double STALE_TX_THRESHOLD  = 0.15;
    private static final int    STALE_TX_MAX_FRAMES = 6;

    // World-angle tracking
    private double  targetWorldAngleDeg    = 0;
    private boolean targetFoundAtLeastOnce = false;
    private boolean worldAngleConfident    = false;

    // Blind scale
    private int blindFrameCount = 0;
    private static final int    BLIND_FULL_POWER_FRAMES  = 15;
    private static final int    BLIND_RAMP_END_FRAMES    = 40;
    private static final double BLIND_MIN_SCALE           = 0.3;
    private static final int    BLIND_CONFIDENCE_EXPIRE   = 90;

    // D-term suppression on reacquisition
    private int framesSinceAcquisition     = 999;
    private static final int DTERM_SUPPRESS_FRAMES = 0;

    // Velocity filter
    private double filteredWorldVelocity       = 0.0;
    private static final double VELOCITY_FILTER_ALPHA = 0.42;

    private double  lastTurretPosDeg  = 0;
    private double  prevWorldAngleDeg = 0;
    private boolean firstUpdate       = true;
    private double  distanceTrack     = 0;

    private static final double MAX_EXPECTED_TX_DRIFT_DEG = 30.0;

    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.150;

    private final ElapsedTime loopTimer  = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

    // Hood / distance
    private static final double LIMELIGHT_HEIGHT = 0.35;
    private static final double LIMELIGHT_ANGLE  = Math.toRadians(15.34);
    private static final double TARGET_HEIGHT     = 0.75;
    private static final double HOOD_MIN          = 0.1;
    private static final double HOOD_MAX          = 0.3;
    private static final double MIN_DISTANCE      = 0.3;
    private static final double MAX_DISTANCE      = 2.5;

    private static final double MIN_RPM = 1400;
    private static final double MAX_RPM = 3300;
    private double shootRPM = MIN_RPM;

    private boolean hasTarget = false;
    private static final double TICKS_PER_DEGREE  = 1.8;
    private static final double TX_ACCEPTANCE_DEG = 35.0;
    private static final double TX_OFFSET_DEG     = 0.0;

    // Unwind
    private boolean unwinding            = false;
    private double  unwindGoalDeg        = 0;
    private double  unwindTriggeredAtDeg = 0;
    private static final double SOFT_LIMIT_CW      =  140.0;
    private static final double SOFT_LIMIT_CCW     = -180.0;
    private static final double UNWIND_MAX_POWER   =  0.90;
    private static final double UNWIND_MIN_POWER   =  0.25;
    private static final double UNWIND_DECEL_DEG   =  20.0;
    private static final double UNWIND_ARRIVE_DEG  =  10.0;

    // Logging
    private BufferedWriter      logWriter      = null;
    private boolean             loggingEnabled = false;
    private final StringBuilder logLine        = new StringBuilder(512);

    private double  lastTy      = 0;
    private double  lastHoodPos = 0;
    private double  odoX        = 0;
    private double  odoY        = 0;
    private double  robotRotVel = 0;
    private boolean autoAimOn   = false;

    private int    loopCount        = 0;
    private int    logFailCount     = 0;
    private int    loopOverrunCount = 0;
    private double lastDeltaMs      = 0;
    private double batteryVoltage   = 0;
    private long   llStalenessMs    = 0;

    // =========================================================================
    // Init
    // =========================================================================

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
        manualMode              = false;
        manualPower             = 0.0;
        unwinding               = false;
        unwindGoalDeg           = 0;
        unwindTriggeredAtDeg    = 0;
        lastCheckedPosDeg       = 0;
        stuckFrameCount         = 0;
        loopTimer.reset();
        totalTimer.reset();

        try {
            String ts   = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String path = "/sdcard/FIRST/turret_log_" + ts + ".csv";
            logWriter   = new BufferedWriter(new FileWriter(path), 65536);
            loggingEnabled = true;
            logWriter.write(
                    "time_ms,raw_tx,smoothed_tx,error,turret_rel_deg,world_angle_deg," +
                            "robot_heading_deg,world_velocity,filtered_velocity,p_term,i_term,d_term,output_power," +
                            "motor_power,has_target,target_world_angle_deg,tx_rejected,stale_tx_count," +
                            "consec_target_frames,blind_frames,confident,blind_scale,shooter_current_rpm," +
                            "shooter_target_rpm,rpm_ready,pusher_fired,distance_m,loop_num,delta_time_ms," +
                            "loop_overrun_total,heap_free_mb,heap_used_mb,battery_v,ll_staleness_ms," +
                            "log_fail_count,ty,hood_pos,odo_x,odo_y,robot_rot_vel,auto_aim_on," +
                            "shooter_ready_reason,stuck_frames\n");
            logWriter.flush();
        } catch (IOException e) {
            loggingEnabled = false;
        }
    }

    public void closeLog() {
        if (logWriter != null) {
            try { logWriter.flush(); logWriter.close(); } catch (IOException ignored) {}
            logWriter = null;
        }
    }

    public void disableLogging() {
        loggingEnabled = false;
        if (logWriter != null) {
            try { logWriter.flush(); logWriter.close(); } catch (IOException e) {}
            logWriter = null;
        }
    }

    // =========================================================================
    // Public API
    // =========================================================================

    public void setMecanumCommand(MecanumCommand mc) { this.mecanumCommand = mc; }
    public void setkP(double v) { this.kP = v; }
    public void setkI(double v) { this.kI = v; }
    public void setkD(double v) { this.kD = v; }
    public double getkP()            { return kP; }
    public double getkI()            { return kI; }
    public double getkD()            { return kD; }
    public double getShootRPM()      { return shootRPM; }
    public double getDistanceTrack() { return distanceTrack; }
    public boolean hasTarget()       { return hasTarget; }
    public double getLastError()     { return prevError; }
    public double getHoodPosition()  { return hood.getPosition(); }
    public boolean isUnwinding()     { return unwinding; }
    public int getStuckFrameCount()  { return stuckFrameCount; }

    public void setManualPower(double power) { manualMode = true;  manualPower = power; }
    public void setAutoMode()                { manualMode = false; }

    // =========================================================================
    // Update — short signature
    // =========================================================================

    public void update(Double tx, Double ty) {
        update(tx, ty, 0, 0, false, false, 0, 0, 0, 0, 0, false);
    }

    // =========================================================================
    // Update — full signature
    // =========================================================================

    public void update(Double tx, Double ty,
                       double shooterCurrentRPM, double shooterTargetRPM,
                       boolean rpmReady, boolean pusherFired,
                       double batteryV, long llStaleMs,
                       double odoX, double odoY,
                       double robotRotVel, boolean autoAimOn) {

        double deltaTime = loopTimer.seconds();
        loopTimer.reset();

        loopCount++;
        lastDeltaMs      = deltaTime * 1000.0;
        batteryVoltage   = batteryV;
        llStalenessMs    = llStaleMs;
        this.odoX        = odoX;
        this.odoY        = odoY;
        this.robotRotVel = robotRotVel;
        this.autoAimOn   = autoAimOn;
        if (lastDeltaMs > 100) loopOverrunCount++;

        if (Double.isNaN(deltaTime) || deltaTime < DT_MIN) deltaTime = DT_MIN;
        else if (deltaTime > DT_MAX)                       deltaTime = DT_MAX;

        double robotHeadingDeg = (mecanumCommand != null)
                ? Math.toDegrees(mecanumCommand.getOdoHeading()) : 0;
        if (Double.isNaN(robotHeadingDeg)) robotHeadingDeg = 0;

        double currentTurretRelDeg  = turret.getCurrentPosition() / TICKS_PER_DEGREE;
        double currentWorldAngleDeg = wrapAngle(robotHeadingDeg + currentTurretRelDeg);

        if (firstUpdate) { prevWorldAngleDeg = currentWorldAngleDeg; firstUpdate = false; }

        double headingJump = Math.abs(wrapAngle(currentWorldAngleDeg - prevWorldAngleDeg));
        if (headingJump > 60.0) {
            currentWorldAngleDeg = prevWorldAngleDeg;
            robotHeadingDeg      = prevWorldAngleDeg - currentTurretRelDeg;
        }

        double currentWorldVelocity = wrapAngle(currentWorldAngleDeg - prevWorldAngleDeg) / deltaTime;
        prevWorldAngleDeg = currentWorldAngleDeg;
        lastTurretPosDeg  = currentTurretRelDeg;

        if (Double.isFinite(currentWorldVelocity) && Math.abs(currentWorldVelocity) <= 400.0) {
            filteredWorldVelocity = VELOCITY_FILTER_ALPHA * currentWorldVelocity
                    + (1.0 - VELOCITY_FILTER_ALPHA) * filteredWorldVelocity;
        }

        if (tx != null) tx = tx - TX_OFFSET_DEG;
        double rawTx       = (tx != null) ? tx : 0.0;
        double error       = 0;
        double outputPower = 0;
        double blindScale  = 1.0;

        // =====================================================================
        // UNWIND
        // =====================================================================
        if (!unwinding && !manualMode) {
            if (currentTurretRelDeg >= SOFT_LIMIT_CW) {
                unwinding = true;
                unwindTriggeredAtDeg = SOFT_LIMIT_CW;
                integralSum = 0; hasTarget = false;
                consecutiveTargetFrames = 0; stableFrameCount = 0;
                stuckFrameCount = 0;
            } else if (currentTurretRelDeg <= SOFT_LIMIT_CCW) {
                unwinding = true;
                unwindTriggeredAtDeg = SOFT_LIMIT_CCW;
                integralSum = 0; hasTarget = false;
                consecutiveTargetFrames = 0; stableFrameCount = 0;
                stuckFrameCount = 0;
            }
        }

        if (unwinding) {
            // Recompute goal every frame so it tracks robot rotation during fast turns
            unwindGoalDeg = computeUnwindGoal(robotHeadingDeg, unwindTriggeredAtDeg);
            double distToGoal  = unwindGoalDeg - currentTurretRelDeg;
            double absDist     = Math.abs(distToGoal);
            boolean nearTarget = (tx != null && Math.abs(tx) < TX_ACCEPTANCE_DEG);

            if (absDist <= UNWIND_ARRIVE_DEG || nearTarget || manualMode) {
                unwinding = false;
                unwindTriggeredAtDeg = 0;
                unwindGoalDeg = 0;
                integralSum = 0;
                prevError = 0;
                smoothedTx = 0;
                prevSmoothedTx = 0;
                stableFrameCount = 0;
                consecutiveTargetFrames = 0;
                framesSinceAcquisition = 999;
                stuckFrameCount = 0;
                filteredWorldVelocity = 0;
            } else {
                double power = (absDist < UNWIND_DECEL_DEG)
                        ? UNWIND_MIN_POWER + (absDist / UNWIND_DECEL_DEG) * (UNWIND_MAX_POWER - UNWIND_MIN_POWER)
                        : UNWIND_MAX_POWER;
                double totalPower = (distToGoal > 0) ? power : -power;
                if (currentTurretRelDeg >= SOFT_LIMIT_CW  && totalPower > 0) totalPower = 0;
                if (currentTurretRelDeg <= SOFT_LIMIT_CCW && totalPower < 0) totalPower = 0;
                turret.setPower(totalPower);
                updateHoodAndRPM(ty);
                return;
            }
        }

        double expectedTx = wrapAngle(currentWorldAngleDeg - targetWorldAngleDeg);

        // Stale-tx detection
        boolean txIsStale = false;
        if (tx != null) {
            if (Math.abs(tx - lastRawTx) < STALE_TX_THRESHOLD
                    && Math.abs(filteredWorldVelocity) > 20.0) staleTxCount++;
            else staleTxCount = 0;
            lastRawTx = tx;
            txIsStale = (staleTxCount >= STALE_TX_MAX_FRAMES);
        } else {
            staleTxCount = 0;
        }

        // Drift-rejection gate
        boolean txRejected = false;
        if (tx != null && worldAngleConfident) {
            if (Math.abs(wrapAngle(tx - expectedTx)) > MAX_EXPECTED_TX_DRIFT_DEG)
                txRejected = true;
        }

        // =====================================================================
        // STATE MACHINE
        // =====================================================================

        if (manualMode) {
            hasTarget               = false;
            consecutiveTargetFrames = 0;
            smoothedTx              = 0.0;
            prevSmoothedTx          = 0.0;
            integralSum             = 0;
            error                   = 0;
            outputPower             = manualPower;
            stuckFrameCount         = 0;

        } else if (tx != null && !txRejected && !txIsStale && Math.abs(tx) < TX_ACCEPTANCE_DEG) {
            targetFoundAtLeastOnce = true;
            consecutiveTargetFrames++;
            blindFrameCount = 0;

            if (!hasTarget) {
                smoothedTx             = worldAngleConfident ? (0.4 * tx + 0.6 * expectedTx) : tx;
                prevSmoothedTx         = smoothedTx;
                integralSum            = integralSum * 0.5;
                stableFrameCount       = 0;
                framesSinceAcquisition = 0;
            } else {
                double errorFactor  = Math.abs(tx) / 12.0;
                double dynamicAlpha = MIN_ALPHA + (MAX_ALPHA - MIN_ALPHA)
                        * Range.clip(errorFactor, 0.0, 1.0);
                smoothedTx = dynamicAlpha * tx + (1.0 - dynamicAlpha) * smoothedTx;
                framesSinceAcquisition++;
            }
            hasTarget = true;

            double txDelta = Math.abs(smoothedTx - prevSmoothedTx);
            prevSmoothedTx = smoothedTx;

            if (worldAngleConfident) {
                if (staleTxCount == 0 && consecutiveTargetFrames >= PIN_UPDATE_MIN_FRAMES) {
                    targetWorldAngleDeg = wrapAngle(currentWorldAngleDeg - smoothedTx);
                }
            } else {
                if (txDelta < TX_STABILITY_THRESHOLD_DEG) {
                    stableFrameCount++;
                    if (stableFrameCount >= STABLE_FRAMES_REQUIRED
                            && consecutiveTargetFrames >= TARGET_FRAMES_REQUIRED) {
                        targetWorldAngleDeg = wrapAngle(currentWorldAngleDeg - smoothedTx);
                        worldAngleConfident = true;
                    }
                } else {
                    stableFrameCount = 0;
                }
            }

            error = -smoothedTx;
            if (Math.abs(error) < ERROR_DEADBAND_DEG) error = 0.0;

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
                    error       = 0;
                    blindScale  = 0;
                } else {
                    error      = -expectedTx;
                    blindScale = getBlindPowerScale();
                }
            } else {
                error      = 0;
                blindScale = 0;
            }
        } else {
            hasTarget               = false;
            consecutiveTargetFrames = 0;
            blindScale              = 0;
        }

        // =====================================================================
        // STUCK DETECTION — flush integral if motor unresponsive
        // =====================================================================
        if (!manualMode) {
            double posDelta = Math.abs(currentTurretRelDeg - lastCheckedPosDeg);
            if (Math.abs(error) > STUCK_ERROR_THRESHOLD) {
                if (posDelta < STUCK_MOVEMENT_DEG) {
                    stuckFrameCount++;
                } else {
                    stuckFrameCount = 0;
                }
            } else {
                // Error is small — we're on target, not stuck
                stuckFrameCount = 0;
            }
            lastCheckedPosDeg = currentTurretRelDeg;

            if (stuckFrameCount >= STUCK_FRAME_THRESHOLD) {
                integralSum     = 0;
                stuckFrameCount = 0;
                RobotLog.ww("Turret", "Stuck detected — integral flushed at " +
                        String.format("%.1f", currentTurretRelDeg) + "°");
            }
        }

        // =====================================================================
        // PID
        // =====================================================================

        double pTerm = 0, iTerm = 0, dTerm = 0;

        if (!manualMode) {
            pTerm = kP * error;

            if (error * prevError < 0) integralSum *= 0.5;
            if (Math.abs(error) < INTEGRAL_SEPARATION_DEG) {
                integralSum += error * deltaTime;
            }
            integralSum = Range.clip(integralSum, -MAX_INTEGRAL / kI, MAX_INTEGRAL / kI);
            iTerm = integralSum * kI;

            if (framesSinceAcquisition >= DTERM_SUPPRESS_FRAMES) {
                dTerm = -filteredWorldVelocity * kD;
            }

            double stictionFF = 0.0;
            double absError   = Math.abs(error);
            if (absError > 0.02) {
                double fadeScale = (absError < MIN_POWER_FADE_WINDOW_DEG)
                        ? absError / MIN_POWER_FADE_WINDOW_DEG : 1.0;
                stictionFF = Math.copySign(FEEDFORWARD_STICTION_POWER * fadeScale, error);
            }

            outputPower = pTerm + iTerm + dTerm + stictionFF;
            if (!hasTarget && blindScale < 1.0) outputPower *= blindScale;
        }

        prevError = error;
        if (Double.isNaN(outputPower)) outputPower = 0;

        double totalPower = Range.clip(outputPower, -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);

        if (currentTurretRelDeg >= SOFT_LIMIT_CW  && totalPower > 0) totalPower = 0;
        if (currentTurretRelDeg <= SOFT_LIMIT_CCW && totalPower < 0) totalPower = 0;

        turret.setPower(totalPower);

        // =====================================================================
        // Logging
        // =====================================================================

        if (loggingEnabled && logWriter != null) {
            try {
                long freeBytes  = Runtime.getRuntime().freeMemory();
                long totalBytes = Runtime.getRuntime().totalMemory();
                logLine.setLength(0);
                logLine.append(totalTimer.milliseconds()).append(',')
                        .append(rawTx).append(',')
                        .append(smoothedTx).append(',')
                        .append(error).append(',')
                        .append(currentTurretRelDeg).append(',')
                        .append(currentWorldAngleDeg).append(',')
                        .append(robotHeadingDeg).append(',')
                        .append(currentWorldVelocity).append(',')
                        .append(filteredWorldVelocity).append(',')
                        .append(pTerm).append(',')
                        .append(iTerm).append(',')
                        .append(dTerm).append(',')
                        .append(outputPower).append(',')
                        .append(totalPower).append(',')
                        .append(hasTarget ? 1 : 0).append(',')
                        .append(targetWorldAngleDeg).append(',')
                        .append(txRejected ? 1 : 0).append(',')
                        .append(staleTxCount).append(',')
                        .append(consecutiveTargetFrames).append(',')
                        .append(blindFrameCount).append(',')
                        .append(worldAngleConfident ? 1 : 0).append(',')
                        .append(getBlindPowerScale()).append(',')
                        .append(shooterCurrentRPM).append(',')
                        .append(shooterTargetRPM).append(',')
                        .append(rpmReady ? 1 : 0).append(',')
                        .append(pusherFired ? 1 : 0).append(',')
                        .append(distanceTrack).append(',')
                        .append(loopCount).append(',')
                        .append(lastDeltaMs).append(',')
                        .append(loopOverrunCount).append(',')
                        .append(freeBytes / 1048576.0).append(',')
                        .append((totalBytes - freeBytes) / 1048576.0).append(',')
                        .append(batteryVoltage).append(',')
                        .append(llStalenessMs).append(',')
                        .append(logFailCount).append(',')
                        .append(lastTy).append(',')
                        .append(lastHoodPos).append(',')
                        .append(this.odoX).append(',')
                        .append(this.odoY).append(',')
                        .append(this.robotRotVel).append(',')
                        .append(this.autoAimOn ? 1 : 0).append(',')
                        .append(!rpmReady       ? "rpm"
                                : !hasTarget      ? "no_target"
                                : Math.abs(prevError) >= 4.0 ? "turret_err"
                                :                   "ready").append(',')
                        .append(stuckFrameCount).append('\n');
                logWriter.write(logLine.toString());
                if ((int)(totalTimer.milliseconds()) % 5000 < 80) logWriter.flush();
            } catch (IOException e) {
                logFailCount++;
                if (logFailCount > 20) loggingEnabled = false;
            }
        }

        updateHoodAndRPM(ty);
    }

    // =========================================================================
    // Unwind goal
    // =========================================================================

    private double computeUnwindGoal(double robotHeadingDeg, double triggerLimit) {
        if (!worldAngleConfident) {
            // No reliable target pin — just go to center
            return 0.0;
        }

        double idealGoal = wrapAngle(targetWorldAngleDeg - robotHeadingDeg);

        // Clamp well inside both limits so we don't immediately re-trigger
        idealGoal = Range.clip(idealGoal, SOFT_LIMIT_CCW + 25, SOFT_LIMIT_CW - 25);

        return idealGoal;
    }

    // =========================================================================
    // Hood / RPM
    // =========================================================================

    private void updateHoodAndRPM(Double ty) {
        if (ty != null) {
            lastTy = ty;
            double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT)
                    / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));
            if (distance < MIN_DISTANCE || distance > 6.0) return;
            distanceTrack = distance;
            double clippedDist = Range.clip(distance * 0.9, MIN_DISTANCE, MAX_DISTANCE);
            double normalized  = (clippedDist - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
            double hoodPos     = HOOD_MAX - Math.pow(normalized, 3) * (HOOD_MAX - HOOD_MIN);
            lastHoodPos        = Range.clip(hoodPos, HOOD_MIN, HOOD_MAX);
            hood.setPosition(lastHoodPos);
            shootRPM = Range.clip(
                    MIN_RPM + Math.pow(normalized, 0.4) * (MAX_RPM - MIN_RPM),
                    MIN_RPM, MAX_RPM);
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

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
            double progress = (double)(blindFrameCount - BLIND_RAMP_END_FRAMES)
                    / (BLIND_CONFIDENCE_EXPIRE - BLIND_RAMP_END_FRAMES);
            return BLIND_MIN_SCALE * (1.0 - Range.clip(progress, 0.0, 1.0));
        }
    }
}