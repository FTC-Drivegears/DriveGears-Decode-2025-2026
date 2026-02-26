package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;

    // ---------------- TURRET PD CONTROL ----------------
    private double kP = 0.02; // slower proportional
    private double kD = 0.0;  // keep derivative for compatibility
    private double goalX = 0;
    private double lastError = 0;
    private final double ANGLE_TOLERANCE = 0.5;
    private final double MAX_POWER = 0.15; // much lower power

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- HOOD / SHOOTER CONSTANTS ----------------
    private final double LIMELIGHT_HEIGHT = 0.35; // meters
    private final double LIMELIGHT_ANGLE = Math.toRadians(25);
    private final double TARGET_HEIGHT = 1.05; // meters
    private final double HOOD_MIN = 0.36;
    private final double HOOD_MAX = 0.75;
    private final double MIN_DISTANCE = 0.5;
    private final double MAX_DISTANCE = 3.0;

    private final double MIN_RPM = 3000;
    private final double MAX_RPM = 4000;
    private double shootRPM = MIN_RPM;

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "llmotor");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hood = hwMap.get(Servo.class, "hood");

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

    /**
     * Updates turret rotation, hood position, and shooter RPM
     */
    public void update(Double tx, Double ty) {

        // ---------------- TURRET ----------------
        if (tx != null) {
            double error = goalX - tx;

            // Simplified PD: proportional + small derivative
            double deltaTime = loopTimer.seconds();
            loopTimer.reset();

            double dTerm = (deltaTime > 0) ? ((error - lastError) / deltaTime) * kD : 0;
            double power = (Math.abs(error) < ANGLE_TOLERANCE) ? 0 : Range.clip(error * kP + dTerm, -MAX_POWER, MAX_POWER);

            turret.setPower(power);
            lastError = error;
        } else {
            turret.setPower(0);
            lastError = 0;
        }

        // ---------------- HOOD & SHOOTER ----------------
        if (ty != null) {

            double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                    Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));

            distance *= 0.85;
            distance = Range.clip(distance, MIN_DISTANCE, MAX_DISTANCE);

            double normalized =
                    (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);

            normalized = Range.clip(normalized, 0, 1);

            // VERY aggressive arc curve
            double hoodCurve = Math.pow(normalized, 3.0);

            double hoodPos =
                    HOOD_MIN + hoodCurve * (HOOD_MAX - HOOD_MIN);

            hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));

            // RPM barely increases
            shootRPM = MIN_RPM + normalized * 300;  // only +300 max

            shootRPM = Range.clip(shootRPM, MIN_RPM, MAX_RPM);
        }
    }
    }
