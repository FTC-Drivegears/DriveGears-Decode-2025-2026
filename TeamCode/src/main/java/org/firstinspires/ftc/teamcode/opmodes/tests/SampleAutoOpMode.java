package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;



@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private int stage1 = 0;

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

        AUTO_STATE autoState = AUTO_STATE.FIRST_BUCKET;
        waitForStart();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();
            //processPinPoint();

            switch (autoState) {
                case FIRST_BUCKET:
                    mecanumCommand.moveToPos(30, 0, 0);
                    if (mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.SUB_PICKUP;
                    }
                    break;
                case SUB_PICKUP:
                    if (mecanumCommand.moveToPos(30, -20, 0)) {
                        autoState = AUTO_STATE.FINISH;
                    }
                    break;
                case FINISH:
                    stopRobot();
                    break;
            }
        }

    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
