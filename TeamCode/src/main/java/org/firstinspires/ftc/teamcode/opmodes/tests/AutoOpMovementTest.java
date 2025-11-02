package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingCommand;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingConstants;


@Autonomous (name = "Movement Test")
public class AutoOpMovementTest extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private SortingCommand sortingCommand;
    private ShooterCommand shooterCommand;

    private double theta;
    private double x;
    private double y;



    private double pos;

    private int stage1 = 0;

    enum AUTO_STATE{
        INITIAL_POS,
        FIRST_LAUNCH,
        COLLECTION,
        SECOND_LAUNCH,

        FINISH
    }

    enum TEST_STATE{
        MOVE,
        TURN
    }

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
        sortingCommand = new SortingCommand(hw);
        shooterCommand = new ShooterCommand(hw);



        waitForStart();
        //unload();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();


            x = mecanumCommand.getOdoX();
            y = mecanumCommand.getOdoY();
            theta = mecanumCommand.getOdoHeading();

            AUTO_STATE autoState = AUTO_STATE.INITIAL_POS;
            SORTING_STATE sortingState = SORTING_STATE.PPG_1;
            TEST_STATE testState = TEST_STATE.MOVE;



//            switch(testState) {
//                case MOVE:
//                    mecanumCommand.moveToPos(100, 0, 0);
//                    break;
//            }



            processTelemetry();
//
//            switch(autoState){
//                case INITIAL_POS:
//                    //robot scans april tag and does the magic mumbo jumbo april tag stuff
//
//                    //sets sortingState
//                    sortingState = SORTING_STATE.PPG_1;
//
//                    mecanumCommand.moveToPos(0, 0, Math.PI/6);
//                    autoState = AUTO_STATE.FIRST_LAUNCH;
//                case FIRST_LAUNCH:
//                    switch(sortingState){
//                        case PPG_1:
//                            //sort to purple, purple, green
//                            break;
//                        case PGP_2:
//                            //sort to purple, green, purple
//                            break;
//                        case GPP_3:
//                            //sort to green, purple, purple
//                            break;
//                    }
//                    //robot pushes
//                    //robot launches
//                    //retract pusher
//                    //power off launcher
//                    autoState = AUTO_STATE.COLLECTION;
//                case COLLECTION:
//                    mecanumCommand.moveToPos(95.55, 90, Math.PI/2);
//                    //turn intake on, somehow find a way to collect all 3
//                    mecanumCommand.moveToPos(95.55, 145, Math.PI/2);
//                    mecanumCommand.moveToPos(95.55, 145, 0.815);
//                    //scan obelisk
//                    sortingState = SORTING_STATE.PPG_1;
//                    autoState = AUTO_STATE.SECOND_LAUNCH;
//
//                case SECOND_LAUNCH:
//                    switch(sortingState){
//                        case PPG_1:
//                            //sort to purple, purple, green
//                            break;
//                        case PGP_2:
//                            //sort to purple, green, purple
//                            break;
//                        case GPP_3:
//                            //sort to green, purple, purple
//                            break;
//                    }
//                    //robot pushes
//                    //robot launches
//                    //retract pusher
//                    //power off launcher
//                    autoState = AUTO_STATE.FINISH;
//
//                case FINISH:
//                    stopRobot();
//                    break;
//
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
