package org.firstinspires.ftc.teamcode.subsystems.odometry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * PinPointOdometrySubsystem wraps GoBildaPinpointDriver.
 *
 * getHeading() returns radians (as the driver provides).
 * DO NOT add Math.toRadians() — the driver already returns radians.
 *
 * CSV log columns:
 *   time_ms          — ms since subsystem init
 *   x_cm             — estimated x position (cm)
 *   y_cm             — estimated y position (cm)
 *   heading_rad      — heading in radians
 *   heading_deg      — heading in degrees (for human readability)
 *   raw_encoder_x    — raw x encoder ticks
 *   raw_encoder_y    — raw y encoder ticks
 *   vel_x            — x velocity (driver units)
 *   vel_y            — y velocity (driver units)
 *   heading_vel      — heading velocity (deg/s from driver)
 *   nan_count        — cumulative NaN readings detected
 *   using_dead_reckon— 1 if this frame used dead reckoning, 0 if sensor valid
 *   loop_dt_ms       — time since last processOdometry() call (ms)
 *   delta_x_cm       — change in x since last frame (cm)
 *   delta_y_cm       — change in y since last frame (cm)
 *   delta_heading_rad— change in heading since last frame (rad)
 *   x_jump_flag      — 1 if |delta_x| > 50cm in one frame (sensor glitch)
 *   h_jump_flag      — 1 if |delta_heading_rad| > 0.5 rad in one frame (glitch)
 */
public class PinPointOdometrySubsystem {

    private GoBildaPinpointDriver pinpointDriver;

    private double x       = 0;
    private double y       = 0;
    private double heading = 0;

    private double previousX       = 0;
    private double previousY       = 0;
    private double previousHeading = 0;

    private double vx     = 0;
    private double vy     = 0;
    private double vtheta = 0;

    private ElapsedTime controllerLoopTime;
    private ElapsedTime totalTimer;
    private int nanCounter = 0;

    // Logging
    private BufferedWriter      logWriter      = null;
    private boolean             loggingEnabled = false;
    private final StringBuilder logLine        = new StringBuilder(256);
    private int                 logFailCount   = 0;
    private int                 frameCount     = 0;

    // Previous frame values for delta computation
    private double prevLogX       = 0;
    private double prevLogY       = 0;
    private double prevLogHeading = 0;

    public PinPointOdometrySubsystem(Hardware hw) {
        pinpointDriver = hw.pinPointOdo;

        pinpointDriver.setOffsets(0, 20);
        pinpointDriver.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        controllerLoopTime = new ElapsedTime();
        totalTimer         = new ElapsedTime();
        pinpointDriver.resetPosAndIMU();
        controllerLoopTime.reset();
        totalTimer.reset();

        try {
            String ts   = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String path = "/sdcard/FIRST/odo_log_" + ts + ".csv";
            logWriter   = new BufferedWriter(new FileWriter(path), 32768);
            loggingEnabled = true;
            logWriter.write(
                    "time_ms,x_cm,y_cm,heading_rad,heading_deg," +
                            "raw_encoder_x,raw_encoder_y," +
                            "vel_x,vel_y,heading_vel," +
                            "nan_count,using_dead_reckon,loop_dt_ms," +
                            "delta_x_cm,delta_y_cm,delta_heading_rad," +
                            "x_jump_flag,h_jump_flag\n");
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

    public int    getNanCounter()       { return nanCounter; }
    public double getControlLoopTime()  { return controllerLoopTime.seconds(); }

    public void processOdometry() {
        double dtMs = controllerLoopTime.milliseconds();
        controllerLoopTime.reset();

        pinpointDriver.update();
        x       = pinpointDriver.getPosX() / 10.0;
        y       = pinpointDriver.getPosY() / 10.0;
        heading = pinpointDriver.getHeading();  // radians — do not convert

        writeLog(dtMs, false);
    }

    public void deadReckoning() {
        double dtMs = controllerLoopTime.milliseconds();
        controllerLoopTime.reset();

        pinpointDriver.update();

        Double checkX       = pinpointDriver.getPosX();
        Double checkY       = pinpointDriver.getPosY();
        Double checkHeading = pinpointDriver.getHeading();

        boolean usedDeadReckon = false;

        if (checkX.isNaN() || checkY.isNaN() || checkHeading.isNaN()) {
            nanCounter++;
            usedDeadReckon = true;
            x       = previousX       + (vx     / 10.0 * dtMs);
            y       = previousY       + (vy     / 10.0 * dtMs);
            heading = previousHeading + (vtheta *        dtMs);
        } else {
            x       =  pinpointDriver.getPosX() / 10.0;
            y       = -pinpointDriver.getPosY() / 10.0;
            heading =  pinpointDriver.getHeading();  // radians — do not convert

            previousX       = x;
            previousY       = y;
            previousHeading = heading;

            vx     = pinpointDriver.getVelX();
            vy     = pinpointDriver.getVelY();
            vtheta = pinpointDriver.getHeadingVelocity();
        }

        writeLog(dtMs, usedDeadReckon);
    }

    private void writeLog(double dtMs, boolean usedDeadReckon) {
        if (!loggingEnabled || logWriter == null) return;
        frameCount++;

        double deltaX   = x       - prevLogX;
        double deltaY   = y       - prevLogY;
        double deltaH   = heading - prevLogHeading;
        // wrap delta heading
        while (deltaH >  Math.PI) deltaH -= 2 * Math.PI;
        while (deltaH < -Math.PI) deltaH += 2 * Math.PI;

        int xJump = Math.abs(deltaX) > 50.0 ? 1 : 0;   // >50cm jump in one frame = glitch
        // Threshold raised 0.5→1.0 rad (57°): PinPoint accumulates heading unboundedly
        // so fast rotation legitimately produces large per-frame deltas at 38ms loop rate.
        // 1.0 rad (~57°) in one frame at 38ms = 1500°/s which is physically impossible.
        int hJump = Math.abs(deltaH) > 1.0 ? 1 : 0;

        prevLogX       = x;
        prevLogY       = y;
        prevLogHeading = heading;

        try {
            logLine.setLength(0);
            logLine.append(totalTimer.milliseconds()).append(',')
                    .append(x).append(',')
                    .append(y).append(',')
                    .append(heading).append(',')
                    .append(Math.toDegrees(heading)).append(',')
                    .append(pinpointDriver.getEncoderX()).append(',')
                    .append(pinpointDriver.getEncoderY()).append(',')
                    .append(vx).append(',')
                    .append(vy).append(',')
                    .append(vtheta).append(',')
                    .append(nanCounter).append(',')
                    .append(usedDeadReckon ? 1 : 0).append(',')
                    .append(dtMs).append(',')
                    .append(deltaX).append(',')
                    .append(deltaY).append(',')
                    .append(deltaH).append(',')
                    .append(xJump).append(',')
                    .append(hJump).append('\n');
            logWriter.write(logLine.toString());
            // Flush every 5s
            if ((int)(totalTimer.milliseconds()) % 5000 < 80) logWriter.flush();
        } catch (IOException e) {
            logFailCount++;
            if (logFailCount > 20) loggingEnabled = false;
        }
    }

    public void setNewPosition(double x, double y, double headingDeg) {
        pinpointDriver.setPosition(
                new Pose2D(DistanceUnit.CM, x, y, AngleUnit.DEGREES, headingDeg));
    }

    public void reset() {
        pinpointDriver.resetPosAndIMU();
        prevLogX = 0; prevLogY = 0; prevLogHeading = 0;
    }

    /**
     * Resets only the XY position to zero while preserving the current heading.
     * Use this for mid-match re-zeroing — avoids corrupting the heading estimate
     * (and therefore the turret's world-angle tracking) the way resetPosAndIMU() does.
     */
    public void resetPositionOnly() {
        double currentHeadingDeg = Math.toDegrees(getHeading());
        pinpointDriver.setPosition(
                new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, currentHeadingDeg));
        prevLogX = 0; prevLogY = 0;
        // prevLogHeading preserved intentionally
    }

    public double getRawX()  { return pinpointDriver.getEncoderX(); }
    public double getRawY()  { return pinpointDriver.getEncoderY(); }

    public double getX()              { return x; }
    public double getY()              { return y; }
    public double getHeading()        { return heading; }  // radians
    public double getHeadingVelocity(){ return pinpointDriver.getHeadingVelocity(); }
    public double getVx()             { return vx; }
    public double getVy()             { return vy; }
    public double getVtheta()         { return vtheta; }
}