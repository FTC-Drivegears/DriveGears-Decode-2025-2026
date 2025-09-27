package org.firstinspires.ftc.teamcode.subsystems.sorting;

import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

import com.qualcomm.robotcore.util.ElapsedTime;
public class SortingCommand {
    private SortingSubsystem sortingSubsystem;
    private Hardware hw;
    private ElapsedTime elapsedTime;

    public SortingCommand(Hardware hw){
        this.hw = hw;
        this.sortingSubsystem = new SortingSubsystem(hw);
        elapsedTime = new ElapsedTime();
    }
    public void changePosition(double degree){
        sortingSubsystem.changePosition(degree);
    }
}

