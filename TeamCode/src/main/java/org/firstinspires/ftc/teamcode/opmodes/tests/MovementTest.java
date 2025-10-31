package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@Autonomous (name = "Movement Test")
public class MovementTest extends LinearOpMode {
    private MecanumCommand mecanumCommand;

    private double theta;
    private double x;
    private double y;



    private double pos;

    private int stage1 = 0;

    enum SORTING_STATE {
        PPG_1,
        PGP_2,
        GPP_3

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        waitForStart();
        //unload();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();
            processTelemetry();
            mecanumCommand.moveToPos(10, 0, 0.0);

            x = mecanumCommand.getOdoX();
            y = mecanumCommand.getOdoY();
            theta = mecanumCommand.getOdoHeading();
            //processPinPoint();


//            switch(sortingState){
//                case PPG_1:
//                    sortingCommand.changePosition(SortingConstants.rotation);
//                    unload();
//                    break;
//
//                case PGP_2:
//                    sortingCommand.changePosition(-SortingConstants.rotation);
//                    unload();
//                    break;
//
//                case GPP_3:
//                    sortingCommand.changePosition(-SortingConstants.rotation);
//                    unload();
//                    break;
//
//            }



//            switch (autoState) {
//                case FIRST_BUCKET:
//                    mecanumCommand.moveToPos(30, 0, 0);
//                    if (mecanumCommand.positionNotReachedYet()) {
//                        autoState = AUTO_STATE.SUB_PICKUP;
//                    }
//                    break;
//                case SUB_PICKUP:
//                    if (mecanumCommand.moveToPos(30, -20, 0)) {
//                        autoState = AUTO_STATE.FINISH;
//                    }
//                    break;
//                case FINISH:
//                    stopRobot();
//                    break;
//            }
        }

    }

    public void processTelemetry(){
        //add telemetry messages here

        telemetry.addLine("---------------------------------");
        telemetry.addData("theta", theta);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();
    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
