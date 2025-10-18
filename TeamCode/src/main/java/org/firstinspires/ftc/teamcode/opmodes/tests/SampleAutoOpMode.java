package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingCommand;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingConstants;


@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private SortingCommand sortingCommand;
    private ShooterCommand shooterCommand;

    private int stage1 = 0;

    enum SORTING_STATE {
        PPG_1,
        PGP_2,
        GPP_3

    }

    public void unload() throws InterruptedException {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        sortingCommand = new SortingCommand(hw);
        shooterCommand = new ShooterCommand(hw);

        SORTING_STATE sortingState = SORTING_STATE.PPG_1;
        waitForStart();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            sortingCommand.changePosition(SortingConstants.rotation);
            //processPinPoint();


            switch(sortingState){
                case PPG_1:
                    sortingCommand.changePosition(SortingConstants.rotation);
                    unload();

                case PGP_2:
                    sortingCommand.changePosition(-SortingConstants.rotation);
                    unload();

                case GPP_3:
                    unload();

            }



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

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
