package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanismTutorial {

    private DcMotorEx turret;

    // PD control gains
    private double kP = 0.01;   // Increased from 0.0001 for Limelight
    private double kD = 0.0;
    private double goalX = 0;   // Target angle / tx
    private double lastError = 0;
    private double angleTolerance = 0.2; // Deadzone

    private final double MAX_POWER = 0.6;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        // Initialize motor, same as before
        turret = hwMap.get(DcMotorEx.class, "llmotor");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetTimer() { timer.reset(); }

    public void setkP(double newkP) { kP = newkP; }
    public void setkD(double newkD) { kD = newkD; }
    public double getkP() { return kP; }
    public double getkD() { return kD; }

    // -------------------------------
    // UPDATE METHOD FOR LIMELIGHT
    // -------------------------------
    // CHANGED: instead of AprilTagDetection curID, now accepts Double tx from Limelight
    public void update(Double tx){
        double deltaTime = timer.seconds();
        timer.reset();

        // CHANGED: tx == null check instead of curID == null
        if (tx == null){
            turret.setPower(0);
            lastError = 0;
            return;
        }

        // CHANGED: error calculation now uses Limelight tx instead of AprilTag bearing
        double error = goalX - tx;

        double pTerm = error * kP;
        double dTerm = 0;

        if(deltaTime > 0){
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance){
            power = 0;
        } else{
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
    }
}
