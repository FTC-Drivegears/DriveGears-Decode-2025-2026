package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

public class TurretMechanismTutorial {

    private DcMotorEx turret;
    private Servo hood;

    private Hardware hw;
    private MecanumCommand mecanumCommand;

    // ---------------- TURRET PD CONTROL ----------------
    private double kP = 0.035;
    private double kD = 0.001;
    private double lastError = 0;
    private final double ANGLE_TOLERANCE = 0.5; // degrees
    private final double MAX_POWER = 0.3; // increase for testing

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------------- HOOD / SHOOTER ----------------
    private final double LIMELIGHT_HEIGHT = 0.35; // meters
    private final double LIMELIGHT_ANGLE = Math.toRadians(25);
    private final double TARGET_HEIGHT = 1.05; // meters
    private final double HOOD_MIN = 0.36;
    private final double HOOD_MAX = 0.75;
    private final double MIN_DISTANCE = 0.3;
    private final double MAX_DISTANCE = 2.0;

    private final double MIN_RPM = 3000;
    private final double MAX_RPM = 4000;
    private double shootRPM = MIN_RPM;

    // ---------------- GOAL POSITION ----------------
    private double goalX = 0; // meters, set your field target X
    private double goalY = 3.0; // meters, set your field target Y

    // ---------------- INITIALIZATION ----------------
    public void init(HardwareMap hwMap, MecanumCommand mecanumCommand) {
        this.hw = Hardware.getInstance(hwMap);
        this.mecanumCommand = mecanumCommand;

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

    /**
     * Update turret rotation using odometry + Limelight as corrector
     */
    public void update(Double tx, Double ty) {
        double deltaTime = loopTimer.seconds();
        deltaTime = Math.max(deltaTime, 0.01); // prevent division by zero
        loopTimer.reset();

        // ---------------- CALCULATE TURRET GOAL ANGLE ----------------
        double robotX = mecanumCommand.getX(); // meters
        double robotY = mecanumCommand.getY(); // meters
        double robotHeading = mecanumCommand.getOdoHeading(); // radians

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double targetAngleField = Math.atan2(dy, dx); // radians
        double targetAngleRobot = normalizeRadians(targetAngleField - robotHeading);
        double goalAngleDeg = Math.toDegrees(targetAngleRobot);

        // ---------------- LIMELIGHT CORRECTION ----------------
        if (tx != null) {
            goalAngleDeg -= tx; // adjust if camera sees target
        }

        // ---------------- PD CONTROL ----------------
        double currentAngleDeg = getTurretAngleDegrees();
        double error = wrapDegrees(goalAngleDeg - currentAngleDeg);
        double dTerm = (error - lastError) / deltaTime * kD;
        double power = (Math.abs(error) < ANGLE_TOLERANCE) ? 0 : Range.clip(error * kP + dTerm, -MAX_POWER, MAX_POWER);

        turret.setPower(power);
        lastError = error;

        // ---------------- HOOD & SHOOTER ----------------
        if (ty != null) {
            double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(ty));
            distance *= 0.85;
            distance = Range.clip(distance, MIN_DISTANCE, MAX_DISTANCE);

            double normalized = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
            normalized = Range.clip(normalized, 0, 1);

            double hoodCurve = Math.pow(normalized, 3.0);
            double hoodPos = HOOD_MIN + hoodCurve * (HOOD_MAX - HOOD_MIN);
            hood.setPosition(Range.clip(hoodPos, HOOD_MIN, HOOD_MAX));

            shootRPM = MIN_RPM + normalized * 300;
            shootRPM = Range.clip(shootRPM, MIN_RPM, MAX_RPM);
        }
    }

    // ---------------- HELPER FUNCTIONS ----------------

    private double getTurretAngleDegrees() {
        int ticks = turret.getCurrentPosition();
        double motorGearRatio = 7.85; // Yellow Jacket
        int ticksPerRev = 28;
        return ticks / (ticksPerRev * motorGearRatio) * 360.0;
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2*Math.PI;
        while (angle < -Math.PI) angle += 2*Math.PI;
        return angle;
    }

    private double wrapDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // ---------------- OPTIONAL: set the goal in meters ----------------
    public void setGoalPosition(double x, double y) {
        goalX = x;
        goalY = y;
    }
}
