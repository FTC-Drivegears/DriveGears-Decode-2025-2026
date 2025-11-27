package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware;

public class ShooterSubsystem {

    private Hardware hw;

    private DcMotorEx shooter;

    private double targetRPM;

    double DEFAULT_RPM = 5000;

    double PPR_of_6000_motor = 28.0;

    double seconds_In_A_Minute = 60.0;

    double kpShooter = 2.0;

    public ShooterSubsystem(Hardware hw) {
        this.hw = hw;
        this.shooter = hw.shooter;
        this.targetRPM = DEFAULT_RPM;
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPositionPIDFCoefficients(kpShooter);
    }

    //returns whether or not we have reached the correctRPM
    public boolean isRPMReached() {
        double currentRPM = hw.shooter.getVelocity() * seconds_In_A_Minute / PPR_of_6000_motor;
        return Math.abs(targetRPM - currentRPM) < 200;
    }

    public boolean spinup(){

        double targetTPS = targetRPM * PPR_of_6000_motor / seconds_In_A_Minute;

        hw.shooter.setVelocity(targetTPS);

        return isRPMReached();
    }

    public void stopShooter(){
        hw.shooter.setVelocity(0);
    }

    public void setMaxRPM(int maxRPM){
        targetRPM = maxRPM;
    }

    public void setMaxRPM(double maxRPM){
        targetRPM = maxRPM;
    }

    public double getShooterVelocity(){
        return hw.shooter.getVelocity();
    }

}
