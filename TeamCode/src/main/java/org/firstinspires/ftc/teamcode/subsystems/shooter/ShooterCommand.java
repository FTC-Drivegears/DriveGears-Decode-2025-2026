package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingSubsystem;

public class ShooterCommand {
    private ShooterSubsystem shooterSubsystem;
    private Hardware hw;
    private ElapsedTime elapsedTime;

    public ShooterCommand(Hardware hw){
        this.hw = hw;
        this.shooterSubsystem = new ShooterSubsystem(hw);
        elapsedTime = new ElapsedTime();
    }
    public void shoot(int seconds) throws InterruptedException {
        shooterSubsystem.shoot(seconds);
    }
    public void stopShoot(int seconds) throws InterruptedException {
        shooterSubsystem.stopShoot(seconds);
    }

    public void eject(){

    }
}
