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
 * TurretMechanism v17.0 — Integral Separation & Approach Braking.
 *
 * Changes from v13.0:
 *  - BLIND_CONFIDENCE_EXPIRE raised 60 → 90: ~2 extra seconds of dead-reckoning
 *  - getBlindPowerScale: replaced hard 0.3 floor with smooth decay to 0 over
 *    frames 40–90, eliminating the authority cliff that caused 147° max error/
 *
 * Changes from v12.0:
 *  - STALE_TX_THRESHOLD raised 0.05 → 0.15: MIN_ALPHA increase caused false
 *    stale detections (66 frames vs 4 in v11); threshold now matches new smoothing rate
 *  - kD raised 0.004 → 0.008: v12 raised kP without proportionally raising kD,
 *    leaving insufficient velocity damping and allowing oscillation to grow
 *  - kP trimmed 0.062 → 0.058: small pullback to reduce overshoot contribution
 *  - MIN_POWER_FADE_WINDOW_DEG widened 1.5 → 3.0: stiction FF was stepping in
 *    abruptly at 1.5°, causing the turret to hunt around zero; wider fade window
 *    gives a smoother onset and lets the D-term settle the final approach
 *  - Integral zero-crossing: halved instead of zeroed; full wipe was creating a
 *    repeating overshoot → wipe → overshoot cycle that looked like oscillation
 */
public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;
    private Hardware hw;
    private MecanumCommand mecanumCommand;

    // --- PID gains ---
    private double kP = 0.050;  // restored from 0.038 — too slow to track moving targets
    private double kI = 0.012;
    private double kD = 0.009;  // was 0.018 — too high: D term spiked to ±3.4 at 190°/s, slamming motor to full reverse and causing D-induced oscillation

    private static final double MAX_INTEGRAL = 0.20;

    // Integral separation: only accumulate when error is small.
    // Prevents windup during approach that fires extra power at the zero-crossing.
    // Diagnosed from logs: mean |i_term| was 0.078 during large errors vs 0.033 small.
    private static final double INTEGRAL_SEPARATION_DEG = 6.0;
    private double integralSum = 0.0;
    private double prevError   = 0.0;

    private static final double FEEDFORWARD_STICTION_POWER = 0.07;  // was 0.12 — FF + P were stacking and overshooting near zero
    private static final double MAX_OUTPUT_POWER = 0.80;

    // Deadband: suppress correction when already very close to target.
    // Prevents micro-oscillations from stiction FF kicking at sub-1-degree errors.
    private static final double ERROR_DEADBAND_DEG = 2.0;  // was 0.7 — overshoot was ±5 deg; 0.7 did nothing

    // Widened 1.5 → 3.0: stiction now fades in smoothly over 3° instead of 1.5°,
    // preventing the abrupt kick that caused hunting near zero
    private static final double MIN_POWER_FADE_WINDOW_DEG = 5.0;  // was 3.0 — stiction now fades in over 5 deg for a gentler approach

    // Manual Mode State
    private boolean manualMode = false;
    private double manualPower = 0.0;

    // Limelight dynamic smoothing bounds
    private static final double MIN_ALPHA = 0.30; // was 0.40 — more smoothing near center reduces tx noise driving oscillation
    private static final double MAX_ALPHA = 1.00;
    private static final double TX_STABILITY_THRESHOLD_DEG = 0.8;
    private double smoothedTx     = 0.0;
    private double prevSmoothedTx = 0.0;

    private int stableFrameCount = 0;
    private static final int STABLE_FRAMES_REQUIRED = 1;
    private int consecutiveTargetFrames = 0;
    private static final int TARGET_FRAMES_REQUIRED = 1;  // was 2 — limelight detection is intermittent (27%), requiring 2 consecutive frames was throwing away 105 valid readings per run

    private double lastRawTx = 0;
    private int staleTxCount = 0;
    // Raised 0.05 → 0.15: MIN_ALPHA=0.40 smooths tx more per frame, so valid
    // readings were triggering the stale gate on the old tight threshold
    private static final double STALE_TX_THRESHOLD = 0.15;
    private static final int STALE_TX_MAX_FRAMES   = 6;

    private double  targetWorldAngleDeg    = 0;
    private boolean targetFoundAtLeastOnce = false;
    private boolean worldAngleConfident    = false;

    // Soft confidence decay
    private int blindFrameCount = 0;
    private static final int BLIND_FULL_POWER_FRAMES = 15;
    private static final int BLIND_RAMP_END_FRAMES   = 40;
    private static final double BLIND_MIN_SCALE       = 0.3;
    // Raised 60 → 90: gives ~2s more dead-reckoning before confidence expires.
    // Combined with smooth decay below, replaces the hard cliff that caused 147° errors.
    private static final int BLIND_CONFIDENCE_EXPIRE  = 90;

    // D-term suppression on reacquisition
    private int framesSinceAcquisition = 999;
    private static final int DTERM_SUPPRESS_FRAMES = 0;  // was 2 — frequent reacquisitions meant D was always suppressed

    // World velocity low-pass filter
    private double filteredWorldVelocity = 0.0;
    private static final double VELOCITY_FILTER_ALPHA = 0.42; // was 0.55 — more smoothing reduces D-term spike magnitude on fast transients

    private double lastTurretPosDeg  = 0;
    private double prevWorldAngleDeg = 0;
    private boolean firstUpdate      = true;
    private double distanceTrack     = 0;

    private static final double MAX_EXPECTED_TX_DRIFT_DEG = 30.0;

    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.150;

    private final ElapsedTime loopTimer  = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

    private static final double LIMELIGHT_HEIGHT = 0.35;
    private static final double LIMELIGHT_ANGLE  = Math.toRadians(15.34);  // calibrated from 2m test: was 40° causing 0.55m reports at actual 2.0m
    private static final double TARGET_HEIGHT     = 0.75;
    private static final double HOOD_MIN          = 0.50;
    private static final double HOOD_MAX          = 0.72;
    private static final double MIN_DISTANCE      = 0.3;
    private static final double MAX_DISTANCE      = 2.5;

    private static final double MIN_RPM = 720;
    private static final double MAX_RPM = 3460;
    private double shootRPM = MIN_RPM;

    private boolean hasTarget = false;
    private static final double TICKS_PER_DEGREE = 1.8;
    private static final double TX_ACCEPTANCE_DEG = 35.0;

    // Crosshair offset — positive shifts aim right, negative shifts aim left.
    // Tune until turret centres on the middle of the target.
    // If turret stops on the LEFT edge  → increase this value
    // If turret stops on the RIGHT edge → decrease (go negative)
    private static final double TX_OFFSET_DEG = 0.0;

    private BufferedWriter logWriter   = null;
    private boolean        loggingEnabled = false;
    // Pre-allocated log buffer — reused every loop to avoid GC pressure from string concatenation
    private final StringBuilder logLine = new StringBuilder(512);

    // Extra log state
    private double  lastTy            = 0;
    private double  lastHoodPos       = 0;
    private double  odoX              = 0;
    private double  odoY              = 0;
    private double  robotRotVel       = 0;
    private boolean autoAimOn         = false;

    // Diagnostic tracking
    private int     loopCount        = 0;
    private int     logFailCount     = 0;
    private int     loopOverrunCount = 0;
    private double  lastDeltaMs      = 0;
    private double  batteryVoltage   = 0;
    private long    llStalenessMs    = 0;

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
        loopTimer.reset();
        totalTimer.reset();

        try {
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String path      = "/sdcard/FIRST/turret_log_" + timestamp + ".csv";
            logWriter        = new BufferedWriter(new FileWriter(path), 65536);  // 64KB buffer reduces flush frequency
            loggingEnabled   = true;
            logWriter.write("time_ms,raw_tx,smoothed_tx,error,turret_rel_deg,world_angle_deg," +
                    "robot_heading_deg,world_velocity,filtered_velocity,p_term,i_term,d_term,output_power," +
                    "motor_power,has_target,target_world_angle_deg,tx_rejected,stale_tx_count," +
                    "consec_target_frames,blind_frames,confident,blind_scale,shooter_current_rpm,shooter_target_rpm,rpm_ready,pusher_fired,distance_m," +
                    "loop_num,delta_time_ms,loop_overrun_total,heap_free_mb,heap_used_mb,battery_v,ll_staleness_ms,log_fail_count," +
                    "ty,hood_pos,odo_x,odo_y,robot_rot_vel,auto_aim_on,shooter_ready_reason\n");
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
    public double getLastError()     { return prevError; }
    public double getHoodPosition()  { return hood.getPosition(); }

    public void setManualPower(double power) {
        this.manualMode = true;
        this.manualPower = power;
    }

    public void setAutoMode() {
        this.manualMode = false;
    }

    private double wrapAngle(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double getBlindPowerScale() {
        if (blindFrameCount <= BLIND_FULL_POWER_FRAMES) {
            return 1.0;
        } else if (blindFrameCount <= BLIND_RAMP_END_FRAMES) {
            // Ramp from 1.0 down to BLIND_MIN_SCALE
            double progress = (double)(blindFrameCount - BLIND_FULL_POWER_FRAMES)
                    / (BLIND_RAMP_END_FRAMES - BLIND_FULL_POWER_FRAMES);
            return 1.0 - progress * (1.0 - BLIND_MIN_SCALE);
        } else {
            // Smooth decay from BLIND_MIN_SCALE to 0 instead of a hard floor.
            // Prevents the sudden authority cliff that caused 147 deg errors in v13.
            double progress = (double)(blindFrameCount - BLIND_RAMP_END_FRAMES)
                    / (BLIND_CONFIDENCE_EXPIRE - BLIND_RAMP_END_FRAMES);
            return BLIND_MIN_SCALE * (1.0 - Range.clip(progress, 0.0, 1.0));
        }
    }

    public void update(Double tx, Double ty) {
        update(tx, ty, 0, 0, false, false, 0, 0, 0, 0, 0, false);
    }

    public void update(Double tx, Double ty, double shooterCurrentRPM, double shooterTargetRPM, boolean rpmReady, boolean pusherFired, double batteryV, long llStaleMs, double odoX, double odoY, double robotRotVel, boolean autoAimOn) {
        double deltaTime = loopTimer.seconds();
        loopTimer.reset();

        loopCount++;
        lastDeltaMs    = deltaTime * 1000.0;
        batteryVoltage = batteryV;
        llStalenessMs  = llStaleMs;
        this.odoX        = odoX;
        this.odoY        = odoY;
        this.robotRotVel = robotRotVel;
        this.autoAimOn   = autoAimOn;
        if (lastDeltaMs > 100) loopOverrunCount++;

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

        // Heading sanity check — if world angle jumps >90° in one frame it is a PinPoint
        // glitch (brownout reset returning 0). Freeze world angle for this frame so a
        // single bad I2C read does not corrupt the world-angle estimate and kill tracking.
        double headingJump = Math.abs(wrapAngle(currentWorldAngleDeg - prevWorldAngleDeg));
        if (headingJump > 90.0) {
            currentWorldAngleDeg = prevWorldAngleDeg;
            robotHeadingDeg      = prevWorldAngleDeg - currentTurretRelDeg;
        }

        double currentWorldVelocity = wrapAngle(currentWorldAngleDeg - prevWorldAngleDeg) / deltaTime;
        prevWorldAngleDeg       = currentWorldAngleDeg;
        lastTurretPosDeg        = currentTurretRelDeg;

        if (Double.isFinite(currentWorldVelocity)) {
            filteredWorldVelocity = VELOCITY_FILTER_ALPHA * currentWorldVelocity
                    + (1.0 - VELOCITY_FILTER_ALPHA) * filteredWorldVelocity;
        }

        double error       = 0;
        double outputPower = 0;
        // Apply crosshair offset so tx=0 means turret is on the target centre
        if (tx != null) tx = tx - TX_OFFSET_DEG;
        double rawTx       = (tx != null) ? tx : 0.0;
        double blindScale  = 1.0;

        double expectedTx = wrapAngle(currentWorldAngleDeg - targetWorldAngleDeg);

        boolean txIsStale = false;
        if (tx != null) {
            if (Math.abs(tx - lastRawTx) < STALE_TX_THRESHOLD) {
                if (Math.abs(filteredWorldVelocity) > 20.0) {
                    staleTxCount++;
                }
            } else {
                staleTxCount = 0;
            }
            lastRawTx = tx;
            txIsStale = (staleTxCount >= STALE_TX_MAX_FRAMES);
        } else {
            staleTxCount = 0;
        }

        boolean txRejected = false;
        if (tx != null && worldAngleConfident) {
            double txError = Math.abs(wrapAngle(tx - expectedTx));
            if (txError > MAX_EXPECTED_TX_DRIFT_DEG) {
                txRejected = true;
            }
        }

        if (manualMode) {
            hasTarget = false;
            consecutiveTargetFrames = 0;
            smoothedTx = 0.0;
            prevSmoothedTx = 0.0;
            integralSum = 0;
            error = 0;
            outputPower = manualPower;
        } else if (tx != null && !txRejected && !txIsStale && Math.abs(tx) < TX_ACCEPTANCE_DEG) {
            targetFoundAtLeastOnce = true;
            consecutiveTargetFrames++;
            blindFrameCount = 0;

            if (!hasTarget) {
                // Blend from expected world-angle toward raw tx on reacquisition.
                // Jumping straight to rawTx caused large first-frame errors and overshoot.
                smoothedTx             = worldAngleConfident ? (0.4 * tx + 0.6 * expectedTx) : tx;
                prevSmoothedTx         = smoothedTx;
                integralSum            = integralSum * 0.5;
                stableFrameCount       = 0;
                framesSinceAcquisition = 0;
            } else {
                double errorFactor  = Math.abs(tx) / 12.0;
                double dynamicAlpha = MIN_ALPHA + (MAX_ALPHA - MIN_ALPHA) * Range.clip(errorFactor, 0.0, 1.0);
                smoothedTx = dynamicAlpha * tx + (1.0 - dynamicAlpha) * smoothedTx;
                framesSinceAcquisition++;
            }
            hasTarget = true;

            double txDelta = Math.abs(smoothedTx - prevSmoothedTx);
            prevSmoothedTx = smoothedTx;

            if (worldAngleConfident) {
                if (staleTxCount == 0) {
                    targetWorldAngleDeg = wrapAngle(currentWorldAngleDeg - smoothedTx);
                }
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

        double pTerm = 0, iTerm = 0, dTerm = 0;

        if (!manualMode) {
            pTerm = kP * error;

            // Halve instead of zero on zero-crossing — zeroing caused a repeating
            // overshoot → wipe → overshoot cycle that manifested as oscillation
            if (error * prevError < 0) integralSum *= 0.5;
            // Only wind up integral when close to target — prevents approach windup overshoot
            if (Math.abs(error) < INTEGRAL_SEPARATION_DEG) {
                integralSum += error * deltaTime;
            }
            integralSum = Range.clip(integralSum, -MAX_INTEGRAL / kI, MAX_INTEGRAL / kI);
            iTerm = integralSum * kI;

            if (framesSinceAcquisition >= DTERM_SUPPRESS_FRAMES) {
                dTerm = -filteredWorldVelocity * kD;
            }

            double stictionFF = 0.0;
            double absError = Math.abs(error);
            if (absError > 0.02) {
                double fadeScale = 1.0;
                if (absError < MIN_POWER_FADE_WINDOW_DEG) {
                    fadeScale = absError / MIN_POWER_FADE_WINDOW_DEG;
                }
                stictionFF = Math.copySign(FEEDFORWARD_STICTION_POWER * fadeScale, error);
            }

            outputPower = pTerm + iTerm + dTerm + stictionFF;

            if (!hasTarget && blindScale < 1.0) {
                outputPower *= blindScale;
            }
        }

        prevError = error;

        if (Double.isNaN(outputPower)) {
            outputPower = 0;
        }

        double totalPower = Range.clip(outputPower, -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);

        // Hard Software Limit Safety Locks
        if (currentTurretRelDeg >= 140.0 && totalPower > 0) {
            totalPower = 0;
        } else if (currentTurretRelDeg <= -203.0 && totalPower < 0) {
            totalPower = 0;
        }

        turret.setPower(totalPower);

        // Logging — uses a pre-allocated StringBuilder to avoid creating ~30 String
        // objects per loop, which was exhausting the heap every 1.5s and causing GC pauses.
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
                        .append(blindScale).append(',')
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
                        .append(odoX).append(',')
                        .append(odoY).append(',')
                        .append(robotRotVel).append(',')
                        .append(autoAimOn ? 1 : 0).append(',')
                        .append(!rpmReady ? "rpm" : !hasTarget ? "no_target" : Math.abs(prevError) >= 4.0 ? "turret_err" : "ready").append('\n');
                logWriter.write(logLine.toString());
                // Flush every 5s instead of every 1s — reduces I/O blocking
                if ((int)(totalTimer.milliseconds()) % 5000 < 80) logWriter.flush();
            } catch (IOException e) { logFailCount++; if (logFailCount > 20) loggingEnabled = false; }
        }

        // Hood and RPM
        if (ty != null) {
            lastTy = ty;
            double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));
            // Sanity check — negative or impossibly large distance means a false limelight detection; ignore it
            if (distance < MIN_DISTANCE || distance > 6.0) return;
            distanceTrack = distance;
            double clippedDist = Range.clip(distance * 0.9, MIN_DISTANCE, MAX_DISTANCE);
            double normalized  = (clippedDist - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
            double hoodPos = 0.72 - Math.pow(normalized, 3) * 0.22;  // close=0.72, far=0.50
            lastHoodPos = Range.clip(hoodPos, HOOD_MIN, HOOD_MAX);
            hood.setPosition(lastHoodPos);
            shootRPM = Range.clip(MIN_RPM + Math.pow(normalized, 0.4) * (MAX_RPM - MIN_RPM), MIN_RPM, MAX_RPM);
        }
    }
}