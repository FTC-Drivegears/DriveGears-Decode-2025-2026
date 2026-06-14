package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware;

/**
 * TurretMechanismAuto
 *
 * Autonomous fixed-hold turret. No Limelight, no world-angle tracking, no
 * stale/drift gates, no blind ramp, no reachability/wedge logic, no logging.
 *
 * You pick a turret-relative angle, an RPM, and a hood position ONCE (setTarget),
 * and update() just runs a simple position-hold PID to pin the turret at that
 * angle for the entire autonomous run. Hood and RPM are constant.
 *
 * Sign convention (unchanged): motor mounted CCW-positive.
 * Position error = goalDeg - currentTurretRelDeg.
 */
public class TurretMechanismAuto {

    private DcMotorEx turret;
    private Servo     hood;
    private Hardware  hw;

    // --- Position-hold PID gains (same family as v18.x) ---
    private double kP = 0.050;
    private double kI = 0.012;
    private double kD = 0.009;

    private static final double MAX_INTEGRAL            = 0.20;
    private static final double INTEGRAL_SEPARATION_DEG = 6.0;
    private double integralSum = 0.0;
    private double prevError   = 0.0;

    private static final double FEEDFORWARD_STICTION_POWER = 0.07;
    private static final double MAX_OUTPUT_POWER           = 0.80;
    private static final double MIN_POWER_FADE_WINDOW_DEG  = 5.0;

    // --- Settle latch with hysteresis (kills limit-cycle shaking) ---
    // Once |error| drops below SETTLE_ENTER_DEG the turret is considered settled:
    // output goes to zero and BRAKE holds it. It only re-engages the PID once
    // |error| grows past SETTLE_EXIT_DEG, so small encoder noise can't restart
    // the back-and-forth nudging.
    private static final double SETTLE_ENTER_DEG = 1.0;
    private static final double SETTLE_EXIT_DEG  = 3.0;
    private boolean settled = false;

    // --- Derivative on measured turret position (low-pass filtered) ---
    private double prevTurretRelDeg     = 0;
    private double filteredTurretVel    = 0.0;
    private static final double VEL_FILTER_ALPHA = 0.30;
    private boolean firstUpdate         = true;

    private static final double DT_MIN = 0.002;
    private static final double DT_MAX = 0.150;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // --- Geometry / limits ---
    private static final double TICKS_PER_DEGREE = 1.8;
    private static final double SOFT_LIMIT_CW    =  140.0;
    private static final double SOFT_LIMIT_CCW   = -180.0;

    // --- Fixed commanded setpoints (chosen once) ---
    private double goalDeg  = 0.0;     // turret-relative target angle
    private double shootRPM = 1400;    // held constant
    private double hoodPos  = 0.20;    // held constant

    private static final double MIN_RPM  = 1400;
    private static final double MAX_RPM  = 3400;
    private static final double HOOD_MIN = 0.05;
    private static final double HOOD_MAX = 0.20;

    // =========================================================================
    // Init
    // =========================================================================

    public void init(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);
        turret  = hw.llmotor;
        hood    = hw.hood;

        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        integralSum       = 0;
        prevError         = 0;
        prevTurretRelDeg  = 0;
        filteredTurretVel = 0;
        settled           = false;
        firstUpdate       = true;
        loopTimer.reset();
    }

    // =========================================================================
    // Setpoints — call once before / at the start of the auto routine
    // =========================================================================

    /**
     * Choose the fixed turret angle, shooter RPM, and hood position to hold for
     * the whole run. Hood is pushed to the servo immediately.
     *
     * @param angleDeg turret-relative target angle (clamped to soft limits)
     * @param rpm      flywheel RPM to hold (clamped MIN_RPM..MAX_RPM)
     * @param hood     hood servo position to hold (clamped HOOD_MIN..HOOD_MAX)
     */
    public void setTarget(double angleDeg, double rpm, double hood) {
        this.goalDeg  = Range.clip(angleDeg, SOFT_LIMIT_CCW, SOFT_LIMIT_CW);
        this.shootRPM = Range.clip(rpm, MIN_RPM, MAX_RPM);
        this.hoodPos  = Range.clip(hood, HOOD_MIN, HOOD_MAX);
        this.hood.setPosition(this.hoodPos);
    }

    public void setGoalDeg(double angleDeg) {
        this.goalDeg = Range.clip(angleDeg, SOFT_LIMIT_CCW, SOFT_LIMIT_CW);
    }

    public void setShootRPM(double rpm) {
        this.shootRPM = Range.clip(rpm, MIN_RPM, MAX_RPM);
    }

    public void setHoodPosition(double hood) {
        this.hoodPos = Range.clip(hood, HOOD_MIN, HOOD_MAX);
        this.hood.setPosition(this.hoodPos);
    }

    public void setkP(double v) { this.kP = v; }
    public void setkI(double v) { this.kI = v; }
    public void setkD(double v) { this.kD = v; }

    public double getShootRPM()      { return shootRPM; }
    public double getHoodPosition()  { return hoodPos; }
    public double getGoalDeg()       { return goalDeg; }
    public double getLastError()     { return prevError; }

    /** True once the turret is settled and holding the goal. */
    public boolean atTarget() { return settled; }

    // =========================================================================
    // Update — call every loop; pure position hold, no sensors but the encoder
    // =========================================================================

    public void update() {
        double deltaTime = loopTimer.seconds();
        loopTimer.reset();
        if (Double.isNaN(deltaTime) || deltaTime < DT_MIN) deltaTime = DT_MIN;
        else if (deltaTime > DT_MAX)                       deltaTime = DT_MAX;

        double currentTurretRelDeg = turret.getCurrentPosition() / TICKS_PER_DEGREE;
        if (firstUpdate) { prevTurretRelDeg = currentTurretRelDeg; firstUpdate = false; }

        // turret velocity (deg/s), low-pass filtered so encoder noise doesn't jitter D
        double rawVel = (currentTurretRelDeg - prevTurretRelDeg) / deltaTime;
        prevTurretRelDeg = currentTurretRelDeg;
        if (!Double.isFinite(rawVel)) rawVel = 0;
        filteredTurretVel = VEL_FILTER_ALPHA * rawVel
                + (1.0 - VEL_FILTER_ALPHA) * filteredTurretVel;

        double error    = goalDeg - currentTurretRelDeg;
        double absError = Math.abs(error);

        // Settle latch with hysteresis: hold zero (BRAKE) inside the band,
        // only re-engage the PID after error grows past the exit band.
        if (settled) {
            if (absError > SETTLE_EXIT_DEG) settled = false;
        } else {
            if (absError < SETTLE_ENTER_DEG) settled = true;
        }

        double totalPower;
        if (settled) {
            // hold position with motor brake; no active drive, no integral creep
            integralSum = 0;
            prevError   = error;
            totalPower  = 0.0;
        } else {
            double pTerm = kP * error;

            if (error * prevError < 0) integralSum *= 0.5;
            if (absError < INTEGRAL_SEPARATION_DEG) {
                integralSum += error * deltaTime;
            }
            integralSum = Range.clip(integralSum, -MAX_INTEGRAL / kI, MAX_INTEGRAL / kI);
            double iTerm = integralSum * kI;

            double dTerm = -filteredTurretVel * kD;

            // stiction kick fades to zero as we approach the settle band so it
            // can't push past the goal and start a limit cycle
            double stictionFF = 0.0;
            if (absError > SETTLE_ENTER_DEG) {
                double fadeScale = (absError < MIN_POWER_FADE_WINDOW_DEG)
                        ? absError / MIN_POWER_FADE_WINDOW_DEG : 1.0;
                stictionFF = Math.copySign(FEEDFORWARD_STICTION_POWER * fadeScale, error);
            }

            double outputPower = pTerm + iTerm + dTerm + stictionFF;
            prevError = error;
            if (Double.isNaN(outputPower)) outputPower = 0;

            totalPower = Range.clip(outputPower, -MAX_OUTPUT_POWER, MAX_OUTPUT_POWER);
        }

        // soft limits
        if (currentTurretRelDeg >= SOFT_LIMIT_CW  && totalPower > 0) totalPower = 0;
        if (currentTurretRelDeg <= SOFT_LIMIT_CCW && totalPower < 0) totalPower = 0;

        turret.setPower(totalPower);

        // hood is constant — keep asserting it in case anything else moved it
        hood.setPosition(hoodPos);
    }

    public void stop() {
        turret.setPower(0);
    }
}