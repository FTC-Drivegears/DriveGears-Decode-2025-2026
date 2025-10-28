package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware;

public class ShooterSubsystem {
    private final Hardware hw;



    public ShooterSubsystem(Hardware hw) {
        this.hw = hw;


    }
    public void shoot(int seconds) throws InterruptedException {
        hw.pusher.setPosition(1);
        //hw.shooter.setPower(1.0);
        Thread.sleep(seconds * 1000);

    }
    public void stopShoot(int seconds) throws InterruptedException {
        hw.pusher.setPosition(0);
        //hw.shooter.setPower(0.0);
        Thread.sleep(seconds * 1000);
    }

}
