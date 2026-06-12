package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware;

public class ShooterSubsystem {

    private Hardware hw;
    private DcMotorEx shooter;
    private double  targetRPM;
    private boolean pidfApplied = false;
    private boolean braking     = false;

    private static final double BRAKE_THRESHOLD_RPM = 150.0;
    private static final double BRAKE_POWER_MAX     = 0.25;

    static final double DEFAULT_RPM          = 2600;
    static final double PPR_OF_6000_MOTOR    = 28.0;
    static final double SECONDS_IN_A_MINUTE  = 60.0;

    private static final double PIDF_P = 10.0;
    private static final double PIDF_I = 0.08;
    private static final double PIDF_D = 2.0;
    private static final double PIDF_F = 14.0;

    private static final double RPM_TOLERANCE_DEFAULT = 100;
    private double rpmTolerance = RPM_TOLERANCE_DEFAULT;

    public ShooterSubsystem(Hardware hw) {
        this.hw        = hw;
        this.shooter   = hw.shooter;
        this.targetRPM = DEFAULT_RPM;

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F));
    }

    /** Set dynamically each loop from distance — close shots get looser tolerance */
    public void setRPMTolerance(double tolerance) {
        rpmTolerance = tolerance;
    }

    public boolean isRPMReached() {
        return Math.abs(targetRPM - getCurrentRPM()) < rpmTolerance;
    }

    public double getCurrentRPM() {
        return hw.shooter.getVelocity() * SECONDS_IN_A_MINUTE / PPR_OF_6000_MOTOR;
    }

    public double getShooterVelocity() {
        return hw.shooter.getVelocity();
    }

    private void applyPIDF() {
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F));
    }

    public boolean spinup() {
        double currentRPM = getCurrentRPM();
        double overshoot  = currentRPM - targetRPM;

        if (overshoot > BRAKE_THRESHOLD_RPM) {
            if (!braking) {
                shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                braking = true;
            }
            double brakePower = Math.min(overshoot / 600.0, 1.0) * BRAKE_POWER_MAX;
            shooter.setPower(-brakePower);
        } else {
            if (braking) {
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
        pidfApplied  = false;
        braking      = false;
        rpmTolerance = RPM_TOLERANCE_DEFAULT;
    }

    public void setMaxRPM(int maxRPM) { targetRPM = maxRPM; }
    public double getTargetRPM()      { return targetRPM; }
    public double getRPMTolerance()   { return rpmTolerance; }
}