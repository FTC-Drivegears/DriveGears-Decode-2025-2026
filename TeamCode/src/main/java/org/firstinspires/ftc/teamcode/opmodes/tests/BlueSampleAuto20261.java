package org.firstinspires.ftc.teamcode.opmodes.tests;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "20261 Blue Auto")
public class BlueSampleAuto20261 extends LinearOpMode {
    private ElapsedTime resetTimer;



    @Override
    public void runOpMode() throws InterruptedException {

        resetTimer = new ElapsedTime();

        waitForStart();

    }

    public void processTelemetry(){
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.update();
    }


}



