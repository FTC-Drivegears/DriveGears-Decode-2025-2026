package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;

    private Hardware hw;

    // ---------------- TURRET PD CONTROL -----------------
    private double kP = 0.035;
    private double kD = 0.001;

    private double distanceTrack;
    private double lastError = 0;
    private final double ANGLE_TOLERANCE = 0.5; // degrees
    private final double MAX_POWER = 0.3;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- HOOD / SHOOTER ----------------
    private final double LIMELIGHT_HEIGHT = 0.31; // meters
    private final double LIMELIGHT_ANGLE = Math.toRadians(40);
    private final double TARGET_HEIGHT = 0.75; // meters
    private final double HOOD_MIN = 0.36;
    private final double HOOD_MAX = 0.75;
    private final double MIN_DISTANCE = 0.3;
    private final double MAX_DISTANCE = 2.0;

    private final double MIN_RPM = 3000;
    private final double MAX_RPM = 4000;
    private double shootRPM = MIN_RPM;

    public void init(HardwareMap hwMap) {
        this.hw = Hardware.getInstance(hwMap);

        turret = hw.llmotor;
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hood = hw.hood;

        loopTimer.reset();
    }

    public void resetTimer() {
        loopTimer.reset();
    }

    public void setkP(double newkP) { kP = newkP; }
    public void setkD(double newkD) { kD = newkD; }
    public double getkP() { return kP; }
    public double getkD() { return kD; }
    public double getShootRPM() { return shootRPM; }
    public double getDistanceTrack() {return distanceTrack;}

    public void update(Double tx, Double ty) {
        double deltaTime = loopTimer.seconds();
        deltaTime = Math.max(deltaTime, 0.01);
        loopTimer.reset();

        // Turret alignment from Limelight only
        if (tx != null) {
            double error = -tx; // flip sign if needed

            double dTerm = ((error - lastError) / deltaTime) * kD;
            double power = (Math.abs(error) < ANGLE_TOLERANCE)
                    ? 0
                    : Range.clip(error * kP + dTerm, -MAX_POWER, MAX_POWER);

            turret.setPower(power);
            lastError = error;
        } else {
            turret.setPower(0);
            lastError = 0;
        }

        // Hood + RPM from Limelight vertical offset
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

            // raise hood a little more for longer shots
            if (distance > 0.55) {
                hoodPos += 0.1;
            }

            hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));

            shootRPM = MIN_RPM + normalized * 300;

            // boost RPM for farther shots
            if (distance > 0.55) {
                shootRPM += 250;
            }

            shootRPM = Range.clip(shootRPM, MIN_RPM, MAX_RPM);
        }
    }

    private double getTurretAngleDegrees() {
        int ticks = turret.getCurrentPosition();
        double motorGearRatio = 7.85;
        int ticksPerRev = 28;
        return ticks / (ticksPerRev * motorGearRatio) * 360.0;
    }

    private double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}