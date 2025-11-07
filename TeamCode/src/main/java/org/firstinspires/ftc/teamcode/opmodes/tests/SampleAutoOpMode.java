package org.firstinspires.ftc.teamcode.opmodes.tests;
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
        INITIAL_POSITION,
        FIRST_SHOT,
        COLLECTION,
        SECOND_SHOT,
        FINISH

    }

    enum PATTERN{
        GPP_1,
        PGP_2,
        PPG_3
    }


    //Pusher variables
    private static final double PUSHER_UP = 0.75;
    private static final double PUSHER_DOWN = 1.0;
    private static final long PUSHER_TIME = 500;
    private static boolean isPusherUp = false;
    private static final ElapsedTime pusherTimer = new ElapsedTime();

    //Sorter variables
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    private static final double SORTER_FIRST_POS = 0.0;
    private static double initialPos = SORTER_FIRST_POS;

    static boolean push(Hardware hw) throws InterruptedException {
        isPusherUp = false;
        hw.pusher.setPosition(PUSHER_DOWN);
        Thread.sleep(10);
        hw.pusher.setPosition(PUSHER_UP);
        isPusherUp = true;
        Thread.sleep(500);
        hw.pusher.setPosition(PUSHER_DOWN);
        isPusherUp = false;
        return isPusherUp;
    }

    static void sort(Hardware hw, int sorterPosition){
        sorterTimer.reset();
        if (sorterPosition == 0) {
            hw.sorter.setPosition(0);
        }
        else if (sorterPosition == 1) {
            hw.sorter.setPosition(0.43);
        }
        else if (sorterPosition == 2) {
            hw.sorter.setPosition(0.875);
        }
    }


    static void shoot(Hardware hw, boolean isOn){
        if(isOn){
            hw.shooter.setPower(1.0);
        }
        else{
            hw.shooter.setPower(0.0);
        }
    }

    static void unload(Hardware hw, int firstpos, int secondpos, int thirdpos) throws InterruptedException{
        int num = 0;
        shoot(hw, true);
        switch (num){
            case 0:
                sort(hw, firstpos);
                num++;
                break;
            case 1:
                if(push(hw)){
                    num++;
                }
                break;
            case 2:
                sort(hw, secondpos);
                num++;
                break;
            case 3:
                if(push(hw)){
                    num++;
                }
                break;
            case 4:
                sort(hw, thirdpos);
                break;
            case 5:
                if (push(hw)){
                    num++;
                }
                break;
        }
        shoot(hw, false);
    }

    static void intake(Hardware hw, boolean isOn){
        if(isOn){
            hw.intake.setPower(0.8);
        }
        else{
            hw.intake.setPower(0.0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        resetTimer = new ElapsedTime();

        hw.sorter.setPosition(initialPos);
        hw.pusher.setPosition(1.0);


        AUTO_STATE autoState = AUTO_STATE.INITIAL_POSITION;
        PATTERN pattern = PATTERN.GPP_1;
        waitForStart();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            processTelemetry();

            hw.pusher.setPosition(0.75);
            Thread.sleep(500);
            hw.pusher.setPosition(1.0);



//            switch (autoState) {
//                case INITIAL_POSITION:
////                    mecanumCommand.moveToPos(turn to position of obelisk);
////                    if(mecanumCommand.isPositionReached()){
////                        scan apriltag
////                        pattern = PATTERN.whatever the pattern is
////                    }
//                    mecanumCommand.moveToPos(0, 0, 0.28447);
//                    if (mecanumCommand.isPositionReached()) {
//                        autoState = AUTO_STATE.FIRST_SHOT;
//                    }
//                case FIRST_SHOT:
//                    switch(pattern){
//                        case GPP_1:
//                            unload(hw, 0, 1, 2);
//                            break;
//                        case PGP_2:
//                            unload(hw, 1, 2, 0);
//                            break;
//                        case PPG_3:
//                            unload(hw, 2, 0, 1);
//                            break;
//                    }
//                    autoState = AUTO_STATE.COLLECTION;
//                    break;
//                case COLLECTION:
//                    intake(hw, true);
//                    int set = 0;
//                    sort(hw, 0);
//                    switch(set){
//                        case 0:
//                            mecanumCommand.moveToPos();
//                            if(mecanumCommand.isPositionReached()){
//                                set++;
//                            }
//                            break;
//                        case 1:
//                            sort(hw, 1);
//                            set++;
//                        case 2:
//                            mecanumCommand.moveToPos();
//                            if(mecanumCommand.isPositionReached()){
//                                set++;
//                            }
//                            break;
//                        case 3:
//                            sort(hw, 2);
//                            set++;
//                        case 4:
//                            mecanumCommand.moveToPos();
//                            break;
//                    }
//                    intake(hw, false);
//                case SECOND_SHOT:
//                    //mecanumCommand.moveToPos(turn to obelisk);
////                    if(mecanumCommand.isPositionReached()){
////                        scan apriltag
////                        pattern = PATTERN.whatever the pattern is
////                    }
//                    //mecanumCommand.moveToPos(launch line);
//
//                    switch(pattern){
//                        case GPP_1:
//                            unload(hw, 0, 1, 2);
//                            break;
//                        case PGP_2:
//                            unload(hw, 1, 2, 0);
//                            break;
//                        case PPG_3:
//                            unload(hw, 2, 0, 1);
//                            break;
//                    }
//                    autoState = AUTO_STATE.FINISH;
//                    break;
//                case FINISH:
//                    mecanumCommand.moveToPos(0, 0 ,0);
//                    stopRobot();
//                    break;
//            }

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
