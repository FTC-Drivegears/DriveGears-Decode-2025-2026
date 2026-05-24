package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware;

public class ShooterSubsystem {

    private Hardware hw;
    private DcMotorEx shooter;
    private double  targetRPM;
    private boolean pidfApplied  = false;  // only set PIDF once per spinup — resetting every loop wipes SDK integral and causes ±259 RPM jitter
    private boolean braking      = false;   // true when actively braking overshoot with RUN_WITHOUT_ENCODER
    private static final double BRAKE_THRESHOLD_RPM = 150.0;  // start braking when this far above target
    private static final double BRAKE_POWER_MAX     = 0.25;   // max reverse power during braking

    // Match turret MIN_RPM default — 5000 was above the turret's calculated range (3000-4300)
    static final double DEFAULT_RPM = 2600;  // lowered from 3000 — motor was overshooting to 3200+ RPM for 45s because default > typical shooting range

    static final double PPR_OF_6000_MOTOR   = 28.0;
    static final double SECONDS_IN_A_MINUTE = 60.0;

    // Max theoretical velocity in ticks/sec for a 6000 RPM / 28 PPR motor:
    // 6000 * 28 / 60 = 2800 ticks/sec
    // F = 32767 / 2800 ≈ 11.7
    // Without F, the motor sits at 0 RPM until PID error accumulates enough
    // to overcome static friction — causing the 40-second spinup seen in logs.
    private static final double PIDF_P = 10.0;
    private static final double PIDF_I = 0.08;  // raised from 0.03 — after brake/shot-drop integral needs to rebuild faster to close residual 150-RPM gap
    private static final double PIDF_D = 2.0;
    private static final double PIDF_F = 14.0;  // raised from 12.5 — too low caused -216 RPM mean undershot; 14.0 middle ground

    // RPM tolerance for isRPMReached — 200 is ~6% at 3300 RPM, reasonable for field use.
    // Tighten to 100 for more accuracy, loosen to 300 if gate feels too slow between shots.
    private static final double RPM_TOLERANCE = 80.0;

    public ShooterSubsystem(Hardware hw) {
        this.hw      = hw;
        this.shooter = hw.shooter;
        this.targetRPM = DEFAULT_RPM;

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF explicitly — SDK default has F=0 which means no feedforward torque.
        // With F=0 the motor waits for PID error to build before moving, causing
        // multi-second (or in our case 40-second) spinup delays on a loaded flywheel.
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F)
        );
    }

    /** Returns true when current RPM is within RPM_TOLERANCE of target. */
    public boolean isRPMReached() {
        double currentRPM = getCurrentRPM();
        return Math.abs(targetRPM - currentRPM) < RPM_TOLERANCE;
    }

    /** Returns current flywheel RPM. */
    public double getCurrentRPM() {
        return hw.shooter.getVelocity() * SECONDS_IN_A_MINUTE / PPR_OF_6000_MOTOR;
    }

    /** Returns raw encoder velocity in ticks/sec (used for logging). */
    public double getShooterVelocity() {
        return hw.shooter.getVelocity();
    }

    /**
     * Reapplies PIDF coefficients to the motor controller.
     * Must be called before every setVelocity() call because the FTC SDK resets
     * motor PIDF to factory defaults (F=0) on every mode transition, including
     * when velocity changes from 0 to non-zero. Without this, F is effectively 0
     * on every spinup, causing the 10-20 second dead zone seen in logs.
     */
    private void applyPIDF() {
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F)
        );
    }

    /**
     * Commands the flywheel to targetRPM every loop.
     * Must be called continuously — not just once on button press.
     * Returns true when RPM is within tolerance.
     */
    public boolean spinup() {
        double currentRPM = getCurrentRPM();
        double overshoot  = currentRPM - targetRPM;

        if (overshoot > BRAKE_THRESHOLD_RPM) {
            // RPM too high — switch to RUN_WITHOUT_ENCODER and apply reverse power
            // proportional to how far above target we are, capped at BRAKE_POWER_MAX
            if (!braking) {
                shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                braking = true;
                // do NOT reset pidfApplied — losing integral on every brake causes
                // 5s stuck episodes as I=0.03 cannot rebuild correction fast enough
            }
            double brakePower = Math.min(overshoot / 600.0, 1.0) * BRAKE_POWER_MAX;
            shooter.setPower(-brakePower);
        } else {
            // Normal velocity control
            if (braking) {
                // Return from braking — re-enter velocity mode and reapply PIDF
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                braking = false;
            }
            if (!pidfApplied) {
                applyPIDF();
                pidfApplied = true;
            }
            double targetTPS = targetRPM * PPR_OF_6000_MOTOR / SECONDS_IN_A_MINUTE;
            shooter.setVelocity(targetTPS);
        }
        return isRPMReached();
    }

    public void stopShooter() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.shooter.setVelocity(0);
        pidfApplied = false;
        braking     = false;
    }

    public void setMaxRPM(int maxRPM) {
        targetRPM = maxRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }
}