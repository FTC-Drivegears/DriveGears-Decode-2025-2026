package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@Autonomous (name = "SampleAutoOpMode")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private FtcDashboard dash;
    private TelemetryPacket packet;
    private int stage1 = 0;

    double[] motorPowers;
    enum AUTO_STATE {
        FIRST_BUCKET,
        SUB_PICKUP,
        FINISH

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);


        boolean firstInstance = true;
        //dash = FtcDashboard.getInstance();
        //telemetry = dash.getTelemetry();
        //packet = new TelemetryPacket();
        ElapsedTime timer;


        AUTO_STATE autoState = AUTO_STATE.FIRST_BUCKET;
        waitForStart();
        while (opModeIsActive()) {
            // run processes
            updateTelemetry();
            motorPowers = mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();


            switch (autoState) {
                case FIRST_BUCKET:
                    if (mecanumCommand.moveToPos(0, 10, 0.0)) {
                        autoState = AUTO_STATE.FINISH;
                    }
                    break;

                case FINISH:
                    stopRobot();
                    break;
            }
        }

    }


    public void updateTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("theta: ", mecanumCommand.getOdoHeading());
        telemetry.addData("motor powers:", motorPowers[1]);
        telemetry.update();
    }



    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }



}

