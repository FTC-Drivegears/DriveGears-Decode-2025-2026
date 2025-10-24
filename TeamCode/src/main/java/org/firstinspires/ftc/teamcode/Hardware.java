package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class Hardware {

    //singleton
    private static Hardware instance;

    // Motors
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;

    public final DcMotorEx intakeMotor;

    public final DcMotorEx outtakeMotor;

    public final Servo outtakeServo;

    public final Servo sorter;

    public final Servo pusher;

    public final CRServo turret;

    // Odometry
    public final GoBildaPinpointDriver pinPointOdo;

    private Hardware(HardwareMap hwMap){
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR); //rightback

        this.pinPointOdo = hwMap.get(GoBildaPinpointDriver.class, Specifications.PIN_POINT_ODOMETRY);

        this.intakeMotor = hwMap.get(DcMotorEx.class, Specifications.INTAKE_MOTOR);
        this.outtakeMotor = hwMap.get(DcMotorEx.class, Specifications.OUTTAKE_MOTOR);

        this.outtakeServo = hwMap.get(Servo.class, Specifications.OUTTAKE_SERVO);
        this.sorter = hwMap.get(Servo.class, Specifications.SORTER);
        this.pusher = hwMap.get(Servo.class, Specifications.PUSHER);
        this.turret = hwMap.get(CRServo.class, Specifications.TURRET);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }
}
