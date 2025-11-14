package org.firstinspires.ftc.teamcode.opmodes.tests;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import java.util.List;

@Autonomous (name = "New Auto")
public class NewAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private ElapsedTime resetTimer;

    enum AUTO_STATE {
        SCAN_OBELISK,
        FIRST_SHOT,
        COLLECTION_1,
        SECOND_SHOT,
        COLLECTION_2,
        THIRD_SHOT,
        FINISH
    }

    enum PATTERN {
        GPP_1, //Tag ID 21
        PGP_2, //Tag ID 22
        PPG_3 //Tag ID 23
    }

    //Pusher variables
    private static final double PUSHER_UP = 0.75;
    private static final double PUSHER_DOWN = 1.0;
    private static final long PUSHER_TIME = 500;
    private static boolean isPusherUp = false;
    private static final ElapsedTime pusherTimer = new ElapsedTime();

    //Sorter variables
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    private static double initialPos = 0.0;

    private static double hoodPos = 1.0;

    private static DcMotor shooter;
    private static Servo pusher;
    private static Servo hood;
    private static Servo sorter;
    private static DcMotor intake;

    private static boolean previousPushState = false;
    private static boolean currentPushState;

    static boolean push() {
        currentPushState = true;
        if (currentPushState && !previousPushState) {
            // Start pulse only if not already pulsing
            if (!isPusherUp) {
                pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
                pusherTimer.reset();
                isPusherUp = true;
            }
        }
        currentPushState = false;
        previousPushState = currentPushState;

        if (isPusherUp && pusherTimer.milliseconds() >= 500) {
            pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
            isPusherUp = false;
        }
        return isPusherUp;
    }


    static void sort(int sorterPosition){

        if (!isPusherUp && sorterTimer.milliseconds() > 1000){
            sorterPosition = (sorterPosition+1)%3;
            sorterTimer.reset();
            if (sorterPosition == 0.0) {
                sorter.setPosition(0.0);//60 degrees
            }
            else if (sorterPosition == 1) {
                sorter.setPosition(0.43);//60 degrees
            }
            else if (sorterPosition == 2) {
                sorter.setPosition(0.875);//60 degrees
            }
        }
    }
    static void shoot(boolean isOn){
        if(isOn){
            shooter.setPower(0.8);
        }
        else{
            shooter.setPower(0.0);
        }
    }

    static void unload (Hardware hw, int firstpos, int secondpos, int thirdpos) {
        int num = 0;
        shoot(true);
        switch (num) {
            case 0:
                sort(firstpos);
                num++;
                break;
            case 1:
                sort(secondpos);
                num++;
                break;
            case 2:
                sort(thirdpos);
                num++;
                break;
        }
    }

//    static void intake(Hardware hw, boolean isOn){
//        if(isOn){
//            intake.setPower(0.8);
//        }
//        else{
//            intake.setPower(0.0);
//        }
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        resetTimer = new ElapsedTime();

        //vision set up
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        shooter = hw.shooter;
        pusher = hw.pusher;
        sorter = hw.sorter;
        hood = hw.hood;
        intake = hw.intake;

        sorter.setPosition(initialPos);
        pusher.setPosition(PUSHER_DOWN);
        hood.setPosition(hoodPos);
//
//        PATTERN pattern = PATTERN.GPP_1;
//        AUTO_STATE autoState = AUTO_STATE.SCAN_OBELISK;


        waitForStart();

//      push();

        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

//            switch (pattern) {
//                case GPP_1:
//                    if (tagProcessor.getDetections().size() > 0) {
//                        AprilTagDetection tag = tagProcessor.getDetections().get(0);
//                        telemetry.addData("ID", tag.id);
//                        processTelemetry();
//
//                    }




            mecanumCommand.moveToPos(50, 75, 0.8); //first intake


            processTelemetry();
                    }
//                    if (AprilTagDetection.tag == 21) {

//            }
//                    if (detection.id == 21) {
////                            pattern = PATTERN.GPP_1;
//                        telemetry.addData("Pattern GPP", detection.id);
//                            autoState = AUTO_STATE.FIRST_SHOT;
//                            break;


//switch (PATTERN) {
////    case GPP_1:
//        if (detection.id == 21) {
//            case GPP_1:
//
//            pattern = PATTERN.GPP_1;
//            telemetry.addData("Pattern: ", DESIRED_TAG_ID);
//            autoState = AUTO_STATE.FIRST_SHOT;
//            break;
////            GPP_1, //Tag ID 21
////            PGP_2, //Tag ID 22
////            PPG_3 //Tag ID 23
//
//}
//            switch (autoState) {
//                case SCAN_OBELISK:
//                    mecanumCommand.moveToPos(0, 0, 0);
//                    List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

//                    autoState = AUTO_STATE.FINISH;

//                    for (AprilTagDetection detection : currentDetections) {
//                        if (detection.id == 21) {
////                            pattern = PATTERN.GPP_1;
//                            telemetry.addData("Pattern GPP", detection.id);
//                            autoState = AUTO_STATE.FIRST_SHOT;
//                            break;

//                        } else if (detection.id == 22) {
//                            pattern = PATTERN.PGP_2;
//                            telemetry.addData("Pattern:", " ");
//                            autoState = AUTO_STATE.FIRST_SHOT;
//                            break;
//
//                        } else if (detection.id == 23) {
//                            pattern = PATTERN.PPG_3;
//                            telemetry.addData("Pattern: ", "PPG_3 (ID 23)");
//                            autoState = AUTO_STATE.FIRST_SHOT;
//                            break;
//                        }
                        }

//                        if (autoState == AUTO_STATE.SCAN_OBELISK) {
//                            telemetry.addLine("No pattern detected. Default: GPP.");
//                            break; //can't detect
//                        }

//                    case FIRST_SHOT:
//                        mecanumCommand.moveToPos(0, 0, 0); //ROTATE TO SHOOT
                        //SORT AND OUTTAKE
//                    shooter.setPower(0.8);
//                    switch(pattern){
//                        case GPP_1:
//                            sort(0);
//                            sort(1);
//                            sort(2);
//                            break;
//                        case PGP_2:
//                            sort(1);
//                            sort(2);
//                            sort(0);
//                            break;
//                        case PPG_3:
//                            sort(2);
//                            sort(0);
//                            sort(1);
//                            break;
//                    }
//                    shooter.setPower(0.0);
//                    autoState = AUTO_STATE.COLLECTION;
//                    break;

// alliance side  *needs to be fixed*
//            if ((desiredTag.id == DESIRED_TAG_ID)) {
//                telemetry.addData("Blue Alliance", desiredTag);
//            }


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





    public void processTelemetry() {
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Theta", mecanumCommand.getOdoHeading());
        telemetry.addData("Position reached: ", mecanumCommand.isPositionReached());
        telemetry.update();
    }
    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
