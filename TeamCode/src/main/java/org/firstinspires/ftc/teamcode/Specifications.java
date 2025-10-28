package org.firstinspires.ftc.teamcode;
//Configuration:
    /*
    Control Hub:

    Motors:
    0:LeftBack
    1:LeftForward
    2:Lift 1
    3:Lift 2

    Servos:
    0. leftLinkage
    2. leftArm
    4. claw

    Expansion Hub:
    0. roll
    1. pitch
    2. light


    Motors:
    0: RightBack
    1: RightForward
    2: Lift 3

    i2c:
    1: pinpoint odo
     */

//laser is in i2c control hub 2 and imu is in 0

public class Specifications {
    public static final String FTLF_MOTOR = "lf";
    public static final String FTRT_MOTOR = "rf";
    public static final String BKLF_MOTOR = "lb";
    public static final String BKRT_MOTOR = "rb";

    public static final String LIME_LIGHT = "lime";

    public static final String LIGHT = "light";

    public static final String PIN_POINT_ODOMETRY = "odo";

    public static final String INTAKE = "intake";

    public static final String OUTTAKE = "outtake";

    public static final String HOOD = "hood";

    public static final String SORTER = "sorter";

    public static final String PUSHER = "pusher";

    public static final String TURRET = "turret";


    public static final String ENCODER = "encoder";
}
