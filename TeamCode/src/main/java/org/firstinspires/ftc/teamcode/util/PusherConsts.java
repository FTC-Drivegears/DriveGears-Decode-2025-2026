package org.firstinspires.ftc.teamcode.util;

public class PusherConsts {
    //    public static final double PUSHER_DOWN_POSITION_R = 0.2;
    public static final double PUSHER_DOWN_POSITION_R = 0.6;
    //    public static final double PUSHER_UP_POSITION_R   = 0.4;
    public static final double PUSHER_UP_POSITION_R   = 0.1;  // was 0.34 — raised for more push; 0.4 snaps

    public static final double PUSHER_DOWN_POSITION_L = 0.0;
    public static final double PUSHER_UP_POSITION_L   = 0.5;  // was 0.35 — raised for more push; 0.4 snaps

    /*
     * Pusher dwell timings (ms).
     *
     * UP/DOWN_TIME        — fast path: ANY mode and the timing test. Floored to
     *                       servo travel (auto-op PUSHER_TIME = 100). Don't drop
     *                       below the physical down->up stroke or it won't complete.
     * UP/DOWN_TIME_SORTED — padded values for sensor-confirmed GREEN/PURPLE firing.
     */
    // 100ms left the servo mid-stroke; raised to 250 so it reaches the full
    // PUSHER_UP_POSITION — the same height Y hits when actually firing (RPM
    // latched). They only look different in the no-shooter timing test, where
    // Y falls back to the 70% preload.
    public static final long PUSHER_UP_TIME_MS          = 250;
    public static final long PUSHER_DOWN_TIME_MS         = 100;
    public static final long PUSHER_UP_TIME_SORTED_MS    = 250;
    public static final long PUSHER_DOWN_TIME_SORTED_MS  = 160; //400
}