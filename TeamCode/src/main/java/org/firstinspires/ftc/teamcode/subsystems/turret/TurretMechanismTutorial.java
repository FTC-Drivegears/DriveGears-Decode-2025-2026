package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TurretMechanismTutorial
 *
 * This subsystem controls:
 * 1. Turret rotation using PD control and Limelight tx values
 * 2. Hood angle adjustment using Limelight ty distance estimation
 *
 * The turret attempts to align the Limelight's horizontal offset (tx) to zero.
 * The hood automatically adjusts based on calculated distance to the target.
 */
public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;

    // ---------------- TURRЕТ PD CONTROL ----------------
    // kP controls how strongly the turret reacts to error
    // kD dampens motion to reduce oscillation
    private double kP = 0.035;
    private double kD = 0.001;

    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;

    private final double MAX_POWER = 0.6;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    // ---------------- HOOD AUTO-AIM CONSTANTS ----------------
    // These values define the physical geometry of the robot + Limelight.
    private final double LIMELIGHT_HEIGHT = 0.35;     // meters
    private final double LIMELIGHT_ANGLE = Math.toRadians(25);
    private final double TARGET_HEIGHT = 0.9;         // meters

    // Servo limits determined experimentally
    private final double HOOD_MIN = 0.359;
    private final double HOOD_MAX = 0.846;

    // Expected shooting distance range (meters)
    private final double MIN_DISTANCE = 0.5;
    private final double MAX_DISTANCE = 3.0;

    /**
     * Initializes turret motor and hood servo from the hardware map.
     */
    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "llmotor");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hood = hwMap.get(Servo.class, "hood");
    }

    public void resetTimer() {
        timer.reset();
    }

    // PD tuning helpers
    public void setkP(double newkP) { kP = newkP; }
    public void setkD(double newkD) { kD = newkD; }
    public double getkP() { return kP; }
    public double getkD() { return kD; }

    /**
     * Main update loop for turret + hood auto-alignment.
     *
     * @param tx horizontal Limelight offset (degrees)
     * @param ty vertical Limelight offset (degrees)
     */
    public void update(Double tx, Double ty) {

        double deltaTime = timer.seconds();
        timer.reset();

        // If no target is detected, stop turret motion
        if (tx == null) {
            turret.setPower(0);
            lastError = 0;
            return;
        }

        // ---------------- TURRET ALIGNMENT ----------------
        double error = goalX - tx;

        double pTerm = error * kP;
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;

        // ---------------- HOOD AUTO-AIM ----------------
        // Distance is estimated using Limelight vertical angle (ty)
        if (ty != null) {

            double distance =
                    (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                            Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));

            // Map distance linearly to hood servo position
            double hoodPos =
                    HOOD_MIN +
                            (distance - MIN_DISTANCE) /
                                    (MAX_DISTANCE - MIN_DISTANCE) *
                                    (HOOD_MAX - HOOD_MIN);

            hoodPos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPos));
            hood.setPosition(hoodPos);
        }
    }
}

