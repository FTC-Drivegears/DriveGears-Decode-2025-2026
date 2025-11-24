package org.firstinspires.ftc.teamcode;

import android.widget.GridLayout;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class Hardware {

    //singleton
    private static Hardware instance;
    // Motors
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;
    public final DcMotorEx intake;
    public final DcMotorEx shooter;
    public final Servo hood;

    public final Servo sorter;
    public final Servo pusher;
    public final IMU imu;
    public Servo light;
    public final CRServo turret;

    public CameraName Webcam_1;

    // Odometry
    public final GoBildaPinpointDriver pinPointOdo;
    public ColorSensor colourSensor2;
    public ColorSensor colourSensor;

    private Hardware(HardwareMap hwMap){
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR); //rightback

        this.pinPointOdo = hwMap.get(GoBildaPinpointDriver.class, Specifications.PIN_POINT_ODOMETRY);
        this.imu = hwMap.get(IMU.class, Specifications.IMU);
        this.intake = hwMap.get(DcMotorEx.class, Specifications.INTAKE);
        this.shooter = hwMap.get(DcMotorEx.class, Specifications.SHOOTER);
        this.colourSensor = hwMap.get(ColorSensor.class, Specifications.COLOUR_SENSOR);
        this.colourSensor2 = hwMap.get(ColorSensor.class, Specifications.COLOUR_SENSOR2);
        this.Webcam_1 = hwMap.get(WebcamName.class, Specifications.WEBCAM_1);

        this.hood = hwMap.get(Servo.class, Specifications.HOOD);
        this.sorter = hwMap.get(Servo.class, Specifications.SORTER);
        this.pusher = hwMap.get(Servo.class, Specifications.PUSHER);
        this.turret = hwMap.get(CRServo.class, Specifications.TURRET);
        this.light = hwMap.get(Servo.class, Specifications.LIGHT);

    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }
}
