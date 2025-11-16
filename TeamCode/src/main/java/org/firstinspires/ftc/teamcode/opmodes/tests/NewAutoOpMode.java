package org.firstinspires.ftc.teamcode.opmodes.tests;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.opmodes.tests.vision.LogitechVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import java.util.List;

@Autonomous (name = "New Auto")
public class NewAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private static ShooterSubsystem shooterSubsystem;
    private ElapsedTime resetTimer;

    private LogitechVisionSubsystem logitechVisionSubsystem;


    private double targetX = Double.NaN; // X position of alliance-specific tag


    enum AUTO_STATE {
        FIRST_SHOT, COLLECTION_1, SECOND_SHOT, COLLECTION_2, THIRD_SHOT, FINISH
    }

    enum PATTERN {
        NO_TAG, //default state
        GPP_1, //Tag ID 21
        PGP_2, //Tag ID 22
        PPG_3 //Tag ID 23
    }

    //Pusher variables
    //Pusher variables
    private static final double PUSHER_UP = 0.65;
    private static final double PUSHER_DOWN = 1.0;
    private static final long PUSHER_TIME = 500;
    private static boolean isPusherUp = false;
    private static final ElapsedTime pusherTimer = new ElapsedTime();

    private static final ElapsedTime stageTimer = new ElapsedTime();

    //Sorter variables
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    private static double pos1 = 0.0;
    private static double pos2 = 0.43;
    private static double pos3 = 0.875;
    private static int standardms = 1000;


    private static double hoodPos = 0.846;

    private static DcMotor shooter;
    private static Servo pusher;
    private static Servo hood;
    private static Servo sorter;
    private static DcMotor intake;

    private static boolean previousPushState = false;
    private static boolean currentPushState;

    private static int stage;


    int detection;

    static boolean halfPush(boolean isUp) {
        if(isUp){
            pusher.setPosition(PUSHER_UP);
            isPusherUp = true;
        }
        else{
            pusher.setPosition(PUSHER_DOWN);
            isPusherUp = false;
        }
        if(pusherTimer.milliseconds() >= 1500){
            pusherTimer.reset();
            return true;
        }
        return false;
    }




    static boolean sort(int sp) {
        if (!isPusherUp){

            if (sp == 0) {
                sorter.setPosition(0.0);//60 degrees
            }
            else if (sp == 1) {
                sorter.setPosition(pos2);//60 degrees
            }
            else if (sp == 2) {
                sorter.setPosition(pos3);//60 degrees
            }
            if (sorterTimer.milliseconds()>= 1500){
                sorterTimer.reset();
                return true;
            }
        }
        return false;
    }

    static void shoot(boolean isOn) {
        if (isOn) {
            shooterSubsystem.spinup();
        } else {
            shooterSubsystem.stopShooter();
        }
    }


    static void intake(boolean isOn) {
        if (isOn) {
            intake.setPower(0.8);
        } else {
            intake.setPower(0.0);
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);

        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        resetTimer = new ElapsedTime();

        shooter = hw.shooter;
        pusher = hw.pusher;
        sorter = hw.sorter;
        hood = hw.hood;
        intake = hw.intake;

        sorter.setPosition(pos1);
        pusher.setPosition(PUSHER_DOWN);
        hood.setPosition(hoodPos);
        int position = 0;
        stage = 0;

        AUTO_STATE autoState = AUTO_STATE.FIRST_SHOT;

        logitechVisionSubsystem = new LogitechVisionSubsystem(hw, "BLUE");
        PATTERN pattern = PATTERN.GPP_1; // default
        boolean outtakeFlag = false;

        telemetry.update();

        long scanStart = System.currentTimeMillis();
        long scanTimeout = 5000;

//        while (!isStarted() && !isStopRequested() &&
//                (System.currentTimeMillis() - scanStart < scanTimeout)) {
        while (!isStarted() && !isStopRequested()) {
            // Detect obelisk pattern
            String detected = logitechVisionSubsystem.pattern();
            String result = "";
            if (detected != null && !detected.equals("UNKNOWN")) {
                switch (detected) {
                    case "GPP_1":
                    case "21":
                        pattern = PATTERN.GPP_1;
                        break;

                    case "PGP_2":
                    case "22":
                        pattern = PATTERN.PGP_2;
                        break;

                    case "PPG_3":
                    case "23":
                        pattern = PATTERN.PPG_3;
                        break;
                }
            }
            targetX = logitechVisionSubsystem.targetApril(telemetry);

            telemetry.addData("Detected Obelisk", detected);
            telemetry.addData("Pattern", pattern);
            telemetry.addData("Target X", targetX);
            telemetry.update();

        }

        waitForStart();



        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();
            shoot(outtakeFlag);


            telemetry.addData("stage", stage);
            telemetry.addData("Pattern", pattern);
            telemetry.addData("position: ", position);
            telemetry.addData("sorterTimer: ", sorterTimer.milliseconds());
            telemetry.addData("stageTimer: ", stageTimer.milliseconds());

            processTelemetry();

            switch (autoState) {
                case FIRST_SHOT:
                    //mecanumCommand.moveToPos(0, 0, Math.PI/9);
                    switch (pattern) {
                        case GPP_1:
                            switch(stage){
                                case 0: //turn on outtake
                                    outtakeFlag = true;
                                    stage++;
                                    stageTimer.reset();
                                    break;
                                case 1: // sort
                                    if(stageTimer.milliseconds() > 1500){
                                        sort(0);
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    }
                                    break;
                                case 2: //push on
                                case 5:
                                case 8:
                                    if (stageTimer.milliseconds() > 750){
                                        halfPush(true);
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    }
                                    break;
                                case 3: //push off
                                case 6:
                                case 9:
                                    if(stageTimer.milliseconds() > 750){
                                        halfPush(false);
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    }
                                    break;
                                case 4: // sort
                                    if(stageTimer.milliseconds() > 1500){
                                        sort(1);
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    }
                                    break;
                                case 7: // sort
                                    if(stageTimer.milliseconds() > 1500){
                                        sort(2);
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    }
                                    break;
                                case 10:
                                    outtakeFlag = false;
                                    stage = 0;
                                    stageTimer.reset();
                                    autoState = AUTO_STATE.COLLECTION_1;
                                    break;
                            }
                            break;
                        case PGP_2:
                            break;
                        case PPG_3:
                            break;
                    }
//                case COLLECTION_1:
//                    intake(true);
//                    int x = 0;
//                    sort(1);
//                    switch (x) {
//                        case 0:
//                            mecanumCommand.moveToPos(0, 0, 0);
//                            x++;
//                            break;
//                        case 1:
//                            sort(0);
//                            mecanumCommand.moveToPos(0, 0, 0);
//                            x++;
//                            break;
//                        case 2:
//                            sort(2);
//                            mecanumCommand.moveToPos(0, 0, 0);
//                            x++;
//                            break;
//                        case 3:
//                            sort(0);
//                            mecanumCommand.moveToPos(0, 0, 0);
//                            break;
//                    }
//                    intake(false);
//                    autoState = AUTO_STATE.SECOND_SHOT;
//                    break;
//
//                case SECOND_SHOT:
//                    mecanumCommand.moveToPos(0, 0, 0);
//                    switch (pattern) {
//                        case GPP_1:
//                            unload(0, 1, 2);
//                            break;
//                        case PGP_2:
//                            unload(1, 2, 0);
//                            break;
//                        case PPG_3:
//                            unload(2, 0, 1);
//                            break;
//
//                    }
//                    autoState = AUTO_STATE.FINISH;
//                    break;
//
//                case FINISH:
//                    mecanumCommand.moveToPos(0, 0, 0);
//                    stopRobot();
//                    break;
//
            }
        }
    }


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

    /// /                    if(mecanumCommand.isPositionReached()){
    /// /                        scan apriltag
    /// /                        pattern = PATTERN.whatever the pattern is
    /// /                    }
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
        telemetry.addData("resetTimer: ", resetTimer.milliseconds());
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
