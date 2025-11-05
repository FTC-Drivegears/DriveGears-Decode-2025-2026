package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;



@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private int stage1 = 0;

    private ElapsedTime resetTimer;

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
        resetTimer = new ElapsedTime();

        AUTO_STATE autoState = AUTO_STATE.FIRST_BUCKET;
        waitForStart();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            processTelemetry();

            switch (autoState) {
                case FIRST_BUCKET:
                    mecanumCommand.moveToPos(50, 50, Math.PI);
                    if (mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.SUB_PICKUP;
                    }
                    break;
                case SUB_PICKUP:
//                    if (mecanumCommand.moveToPos(30, -20, 0)) {
//                        autoState = AUTO_STATE.FINISH;
//                    }
//                    break;
                case FINISH:
                    stopRobot();
                    break;
            }
        }

    }
    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Theta", mecanumCommand.getOdoHeading());
        telemetry.update();
    }
    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
