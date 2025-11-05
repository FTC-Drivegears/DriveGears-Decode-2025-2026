package org.firstinspires.ftc.teamcode.subsystems.sorting;


import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware;

public class SortingSubsystem {
    private final Hardware hw;

    public double position = 0.0;

    public SortingSubsystem(Hardware hw) {
        this.hw = hw;
    }
    public void changePosition(double degree){
        position += degree;
        hw.sorter.setPosition(position);
    }
    public void setPosition(double degree){
        position = degree;
        hw.sorter.setPosition(degree);
    }

}

