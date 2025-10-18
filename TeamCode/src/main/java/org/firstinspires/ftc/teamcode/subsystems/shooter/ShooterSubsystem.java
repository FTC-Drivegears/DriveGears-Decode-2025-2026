package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware;

public class ShooterSubsystem {
    private final Hardware hw;



    public ShooterSubsystem(Hardware hw) {
        this.hw = hw;


    }
    public void shoot() throws InterruptedException {
        hw.pusher.setPosition(0.33);
        hw.shooter.setPower(0.6);
        Thread.sleep(2000);
        hw.shooter.setPower(0.0);
        hw.pusher.setPosition(0.0);


    }
}
