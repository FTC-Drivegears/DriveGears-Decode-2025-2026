package org.firstinspires.ftc.teamcode.subsystems.sorting;


import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware;

public class SortingSubsystem {
    private final Hardware hw;

    public double position = 0.0;

    public SortingSubsystem(Hardware hw) {
        this.hw = hw;

        hw.sorting.setPosition(0.0);
    }
    public void changePosition(double degree){
        position += degree;
        if (position >= 1.0){
            position = 0.0;
        }
        hw.sorting.setPosition(position);
    }

}

