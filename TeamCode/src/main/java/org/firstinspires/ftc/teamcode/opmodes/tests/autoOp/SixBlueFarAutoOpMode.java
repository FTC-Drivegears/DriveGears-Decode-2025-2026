package org.firstinspires.ftc.teamcode.opmodes.tests.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.opmodes.tests.vision.LogitechVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

@Autonomous (name = "Six Blue Auto")
public class SixBlueFarAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private static ShooterSubsystem shooterSubsystem;
    private ElapsedTime resetTimer;

    private LogitechVisionSubsystem logitechVisionSubsystem;


    private double targetX = Double.NaN; // X position of alliance-specific tag


    enum AUTO_STATE {
        FIRST_SHOT, RESET, COLLECTION_1, SECOND_SHOT, COLLECTION_2, THIRD_SHOT, FINISH
    }

    enum PATTERN {
        GPP_1, //Tag ID 21
        PGP_2, //Tag ID 22
        PPG_3 //Tag ID 23
    }

    //Pusher variables
    //Pusher variables
    private static final double PUSHER_UP = 0.0;
    private static final double PUSHER_DOWN = 1.0;
    private static final long PUSHER_TIME = 500;
    private static boolean isPusherUp = false;
    private static final ElapsedTime pusherTimer = new ElapsedTime();

    private static final ElapsedTime stageTimer = new ElapsedTime();

    //Sorter variables
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    private static double pos1 = 0.085;
    private static double pos2 = 0.515;
    private static double pos3 = 0.96;
    private static int standardms = 1000;


    private static double hoodPos = 0.359;

    private static DcMotor shooter;
    private static Servo pusher;
    private static Servo hood;
    private static Servo sorter;
    private static DcMotorEx intake;

    private static boolean previousPushState = false;
    private static boolean currentPushState;

    private static int stage;
    PATTERN pattern = PATTERN.PPG_3; // default


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
                sorter.setPosition(pos1);//60 degrees
                return true;
            }
            else if (sp == 1) {
                sorter.setPosition(pos2);//60 degrees
                return true;
            }
            else if (sp == 2) {
                sorter.setPosition(pos3);//60 degrees
                return true;
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
            intake.setPower(-0.8);
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
        boolean outtakeFlag = false;
        boolean intakeFlag = false;


        telemetry.update();

        long scanStart = System.currentTimeMillis();
        long scanTimeout = 5000;

        // Detect obelisk pattern
        while (!isStarted() && !isStopRequested()) {
            String detected = logitechVisionSubsystem.pattern();
            String result = "";
            if (detected != null && !detected.equals("UNKNOWN")) {
                switch (detected) {
                    case "Obelisk_GPP":
                    case "21":
                        pattern = PATTERN.GPP_1;
                        break;

                    case "Obelisk_PGP":
                    case "22":
                        pattern = PATTERN.PGP_2;
                        break;

                    case "Obelisk_PPG":
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
            intake(intakeFlag);

            telemetry.addData("stage", stage);
            telemetry.addData("Pattern", pattern);
            telemetry.addData("position: ", position);
            telemetry.addData("sorterTimer: ", sorterTimer.milliseconds());
            telemetry.addData("stageTimer: ", stageTimer.milliseconds());

            processTelemetry();



            switch (autoState) {
                case FIRST_SHOT:
                    shooterSubsystem.setMaxRPM(3800);
                    intakeFlag = true;
                    //mecanumCommand.moveToPos(26, -14, 0.5014);
                    mecanumCommand.moveToPos(26, -6, 0.4014);
                    hood.setPosition(0.43); //replace with hood position
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1:
                                switch (stage) {
                                    case 0: //turn on outtake
                                        outtakeFlag = true;
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    case 1: // sort
                                        if (stageTimer.milliseconds() > 1500) {
                                            sort(0);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 2: //push on
                                    case 5:
                                    case 8:
                                        if (stageTimer.milliseconds() > 750 && shooterSubsystem.isRPMReached()) {
                                            halfPush(true);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 3: //push off
                                    case 6:
                                    case 9:
                                        if (stageTimer.milliseconds() > 750) {
                                            halfPush(false);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 4: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(1);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 7: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(2);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 10:
                                        stage = 0;
                                        stageTimer.reset();
                                        autoState = AUTO_STATE.RESET;
                                        break;
                                }
                                break;
                            case PGP_2:
                                switch (stage) {
                                    case 0: //turn on outtake
                                        outtakeFlag = true;
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    case 1: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(2);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 2: //push on
                                    case 5:
                                    case 8:
                                        if (stageTimer.milliseconds() > 750 && shooterSubsystem.isRPMReached()) {
                                            halfPush(true);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 3: //push off
                                    case 6:
                                    case 9:
                                        if (stageTimer.milliseconds() > 750) {
                                            halfPush(false);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 4: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(0);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 7: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(1);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 10:
                                        stage = 0;
                                        stageTimer.reset();
                                        autoState = AUTO_STATE.RESET;
                                        break;
                                }
                                break;
                            case PPG_3:
                                switch (stage) {
                                    case 0: //turn on outtake
                                        outtakeFlag = true;
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    case 1: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(1);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 2: //push on
                                    case 5:
                                    case 8:
                                        if (stageTimer.milliseconds() > 750 && shooterSubsystem.isRPMReached()) {
                                            halfPush(true);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 3: //push off
                                    case 6:
                                    case 9:
                                        if (stageTimer.milliseconds() > 750) {
                                            halfPush(false);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 4: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(2);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 7: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(0);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 10:
                                        stage = 0;
                                        stageTimer.reset();
                                        autoState = AUTO_STATE.RESET;
                                        break;
                                }
                                break;
                        }
                        break;

                    }
                    break;
                case RESET: //set position for ball 1
                    if(!isPusherUp && stageTimer.milliseconds() > 500){
                        if(sort(0)) {
                            stageTimer.reset();
                            autoState = AUTO_STATE.COLLECTION_1;
                        }
                    }
                    break;

                case COLLECTION_1:
                    switch (stage) {
                        case 0: //align with artifacts
                            mecanumCommand.moveToPos(80, 32, Math.PI / 2); //align with artifacts
                            stageTimer.reset();
                            stage++;
                            break;
                        case 1: //turn on intake
                            if (mecanumCommand.isPositionReached()) {
                                stage++;
                                stageTimer.reset();
                            }
                            break;
                        case 2: //intake first ball
                            if (stageTimer.milliseconds() > 800) { //replace with whatever time you think is appropriate
                                mecanumCommand.moveToPos(80, 45, Math.PI / 2); //go to place to intake first artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 3: //set position for second ball
                            if (stageTimer.milliseconds() > 1250) { //replace with whatever time you think is appropriate
                                sort(1);
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 4: //intake second ball
                            if (stageTimer.milliseconds() > 1500) { //replace with whatever time you think is appropriate
                                mecanumCommand.moveToPos(80, 58, Math.PI / 2); //go to place to intake second artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 5: //set position to third ball
                            if (stageTimer.milliseconds() > 1000) { //replace with whatever time you think is appropriate
                                sort(2);
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 6: //move to third ball
                            if (stageTimer.milliseconds() > 1000) { //replace with whatever time you think is appropriate
                                mecanumCommand.moveToPos(80, 85, Math.PI / 2); //go to place to intake third artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 7:
                            if (stageTimer.milliseconds() > 1000) { //replace with whatever time you think is appropriate
                                intakeFlag = false;
                                stageTimer.reset();
                                stage = 0;
                                autoState = AUTO_STATE.SECOND_SHOT;
                                break;
                            }
                            break;

                    }
                    break;

                case SECOND_SHOT:
                    //mecanumCommand.moveToPos(26, -14, 0.5014); //move to whatever position we used to go to
                    mecanumCommand.moveToPos(26, -6, 0.4014);
                    shooterSubsystem.setMaxRPM(3800);
                    hood.setPosition(0.43); //replace with hood position
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1:
                                switch (stage) {
                                    case 0: //turn on outtake
                                        outtakeFlag = true;
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    case 1: // sort
                                        if (stageTimer.milliseconds() > 800) {
                                            sort(0);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 2: //push on
                                    case 5:
                                    case 8:
                                        if (stageTimer.milliseconds() > 750 && shooterSubsystem.isRPMReached()) {
                                            halfPush(true);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 3: //push off
                                    case 6:
                                    case 9:
                                        if (stageTimer.milliseconds() > 750) {
                                            halfPush(false);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 4: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(1);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 7: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(2);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 10:
                                        outtakeFlag = false;
                                        stage = 0;
                                        stageTimer.reset();
                                        autoState = AUTO_STATE.FINISH;
                                        break;
                                }
                                break;
                            case PGP_2:
                                switch (stage) {
                                    case 0: //turn on outtake
                                        outtakeFlag = true;
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    case 1: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(2);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 2: //push on
                                    case 5:
                                    case 8:
                                        if (stageTimer.milliseconds() > 750 && shooterSubsystem.isRPMReached()) {
                                            halfPush(true);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 3: //push off
                                    case 6:
                                    case 9:
                                        if (stageTimer.milliseconds() > 750) {
                                            halfPush(false);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 4: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(0);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 7: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(1);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 10:
                                        outtakeFlag = false;
                                        stage = 0;
                                        stageTimer.reset();
                                        autoState = AUTO_STATE.FINISH;
                                        break;
                                }
                                break;
                            case PPG_3:
                                switch (stage) {
                                    case 0: //turn on outtake
                                        outtakeFlag = true;
                                        stage++;
                                        stageTimer.reset();
                                        break;
                                    case 1: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(1);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 2: //push on
                                    case 5:
                                    case 8:
                                        if (stageTimer.milliseconds() > 750 && shooterSubsystem.isRPMReached()) {
                                            halfPush(true);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 3: //push off
                                    case 6:
                                    case 9:
                                        if (stageTimer.milliseconds() > 750) {
                                            halfPush(false);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 4: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(2);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 7: // sort
                                        if (stageTimer.milliseconds() > 1250) {
                                            sort(0);
                                            stage++;
                                            stageTimer.reset();
                                        }
                                        break;
                                    case 10:
                                        outtakeFlag = false;
                                        stage = 0;
                                        stageTimer.reset();
                                        autoState = AUTO_STATE.FINISH;
                                        break;
                                }
                                break;
                        }
                        break;
                    }
                    break;
                case FINISH:
                    mecanumCommand.moveToPos(60, 0, 0); //replace with box position
                    mecanumCommand.stop();
                    break;

            }
        }
    }


    public void processTelemetry() {
        //add telemetry messages here
        telemetry.addData("resetTimer: ", resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Theta", mecanumCommand.getOdoHeading());
        telemetry.addData("Shooter: ", shooterSubsystem.getShooterVelocity());
        telemetry.update();
    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}