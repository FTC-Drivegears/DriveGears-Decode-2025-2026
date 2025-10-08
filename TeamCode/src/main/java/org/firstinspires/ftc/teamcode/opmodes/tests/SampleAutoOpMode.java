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

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw, telemetry);

        boolean firstInstance = true;

        ElapsedTime timer;
        waitForStart();
        while (opModeIsActive()) {
            // run processes
            //updateTelemetry();
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            mecanumCommand.moveToPos(0, 100, 0.0);

//            switch (autoState) {
//                case FIRST_BUCKET:
//                    telemetry.addLine("running");
//                    if (mecanumCommand.moveToPos(100, 100, 0.0)) {
//
//                        autoState = AUTO_STATE.FINISH;
//                    }
//                    break;
//
//                case FINISH:
//                    stopRobot();
//                    telemetry.addLine("finished");
//                    break;
//            }
        }
    }


//    public void updateTelemetry() {
//        packet.put("x: ", mecanumCommand.getOdoX());
//        packet.put("y: ", mecanumCommand.getOdoY());
//        packet.put("theta: ", mecanumCommand.getOdoHeading());
//    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }

}

