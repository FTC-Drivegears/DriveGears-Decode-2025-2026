package org.firstinspires.ftc.teamcode.opmodes.tests.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.opmodes.tests.vision.LogitechVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

@Autonomous (name = "Red Auto")
public class RedFarAutoOp extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private static ShooterSubsystem shooterSubsystem;

    private static SorterSubsystem sorterSubsystem;
    private ElapsedTime resetTimer;

    private LogitechVisionSubsystem logitechVisionSubsystem;


    private double targetX = Double.NaN; // X position of alliance-specific tag


    enum AUTO_STATE {
        FIRST_SHOT, RESET, COLLECTION_1, SECOND_SHOT, RESET_2, COLLECTION_2, THIRD_SHOT, FINISH
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
    private static final long PUSHER_TIME = 150;
    private static boolean isPusherUp = false;
    private static final ElapsedTime pusherTimer = new ElapsedTime();

    private static final ElapsedTime stageTimer = new ElapsedTime();

    //Sorter variables
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    private static double pos1 = 0.085;
    private static double pos2 = 0.515;
    private static double pos3 = 0.96;
    private static int standardms = 1000;

    private static final long SORTER_TIME = 250;
    private static int currentSort = -1;

    private static final ElapsedTime intakeTimer = new ElapsedTime();
    private static final long INTAKE_WAIT = 700;
    private static boolean intakeWasOn = false;

    private static int collectedCount = 0;


    private static double hoodPos = 0.359;

    private static final double GATE_UP = 1.0;
    private static final double GATE_DOWN = 0.675;
//
//    private double initialHeading = 0.0;
//    private int initialTurretPos = 0;
//    private static final double TICKS_PER_RAD = 100.0;

    private static DcMotor shooter;
    private static Servo pusher;
    private static Servo hood;
    private static Servo sorter;
    private static Servo gate;
    private static DcMotorEx intake;
    private static DcMotorEx turret;


    private static boolean previousPushState = false;
    private static boolean currentPushState;

    boolean outtakeFlag = false;
    boolean intakeFlag = false;
    AUTO_STATE autoState = AUTO_STATE.FIRST_SHOT;

    private static int stage;
    PATTERN pattern = PATTERN.PPG_3; // default


    int detection;

    static boolean halfPush(boolean isUp) {
        if (isUp) {
            if (!isPusherUp) {
                pusher.setPosition(PUSHER_UP);
                isPusherUp = true;
                pusherTimer.reset(); // start timing physical move
            }
        } else {
            if (isPusherUp) {
                pusher.setPosition(PUSHER_DOWN);
                isPusherUp = false;
                pusherTimer.reset(); // start timing physical move
            }
        }
        return pusherTimer.milliseconds() >= PUSHER_TIME;
    }


    static boolean sort(int sp) {
        double pos = (sp == 0) ? pos1 : (sp == 1) ? pos2 : pos3;
        sorter.setPosition(pos);
        currentSort = sp;
        sorterTimer.reset();
        return true;
    }

    static boolean sort() {
        if (sorterTimer.milliseconds() > 500) {
            sorterTimer.reset();
            sorterSubsystem.manualSpin();
            return true;
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
            intake.setPower(-1.0);
            if (!intakeWasOn) {
                intakeTimer.reset();
                intakeWasOn = true;
            }
        } else {
            intake.setPower(0.0);
            intakeWasOn = false;
        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);

        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "");
        resetTimer = new ElapsedTime();

        shooter = hw.shooter;
        pusher = hw.pusher;
        sorter = hw.sorter;
        hood = hw.hood;
        gate = hw.gate;
        intake = hw.intake;
        turret = hw.llmotor;

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sorter.setPosition(pos1);
        pusher.setPosition(PUSHER_DOWN);
        hood.setPosition(hoodPos);
        gate.setPosition(GATE_DOWN);
        int position = 0;
        stage = 0;

//        initialHeading = mecanumCommand.getOdoHeading();
//        initialTurretPos = turret.getCurrentPosition();
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        logitechVisionSubsystem = new LogitechVisionSubsystem(hw, "BLUE");



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
            turret.setPower(1.0);
            shoot(outtakeFlag);
            intake(intakeFlag);
            telemetry.addData("stage", stage);
            telemetry.addData("Pattern", pattern);
            telemetry.addData("position: ", position);
            telemetry.addData("sorterTimer: ", sorterTimer.milliseconds());
            telemetry.addData("stageTimer: ", stageTimer.milliseconds());

//            double curHeading = mecanumCommand.getOdoHeading();
//            double headingDelta = curHeading - initialHeading;
//            int desiredTurretPos = initialTurretPos - (int)Math.round(headingDelta * TICKS_PER_RAD);
//            turret.setTargetPosition(desiredTurretPos);
//            turret.setPower(0.4);



            processTelemetry();

            switch (autoState) {
                case FIRST_SHOT:
                    shooterSubsystem.setMaxRPM(3500);
                    //mecanumCommand.moveToPos(26, -14, 0.5014);
                    mecanumCommand.moveToPos(26, 6, -0.36);
                    hood.setPosition(0.43); //replace with hood position
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1:
                                processGPP1(AUTO_STATE.RESET);
                                break;
                            case PGP_2:
                                processPGP2(AUTO_STATE.RESET);
                                break;
                            case PPG_3:
                                processPPG3(AUTO_STATE.RESET);
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
                            stage = 0;
                            intakeFlag = true;
                        }
                    }
                    break;

                case COLLECTION_1:
                    switch (stage) {
                        case 0: //align with artifacts
                            mecanumCommand.moveToPos(82, -32, -Math.PI / 2); //align with artifacts
                            gate.setPosition(GATE_UP);
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
                            if (stageTimer.milliseconds() > 500) { //replace with whatever time you think is appropriate
                                mecanumCommand.moveToPos(82, -48, -Math.PI / 2); //go to place to intake first artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 3: //set position for second ball
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) { //replace with whatever time you think is appropriate
                                if(sort(1)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 4: //intake second ball
                            if (stageTimer.milliseconds() > 750) { //replace with whatever time you think is appropriate
                                mecanumCommand.moveToPos(82, -63, -Math.PI / 2); //go to place to intake second artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 5: //set position to third ball
                            if (stageTimer.milliseconds() > 750 && intakeTimer.milliseconds() >= INTAKE_WAIT) { //replace with whatever time you think is appropriate
                                if(sort(2)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 6:
                            if (stageTimer.milliseconds() > 500) { //replace with whatever time you think is appropriate
                                stageTimer.reset();
                                stage = 0;
                                autoState = AUTO_STATE.SECOND_SHOT;
                                shooterSubsystem.setMaxRPM(3500);
                                //mecanumCommand.moveToPos(26, -14, 0.5014);
                                mecanumCommand.moveToPos(26, 6, -0.36);
                                hood.setPosition(0.43); //replace with hood position
                                break;
                            }
                            break;

                    }
                    break;

                case SECOND_SHOT:
                    if (mecanumCommand.isPositionReached()) {
                        intakeFlag = false;
                        gate.setPosition(GATE_DOWN);
                        switch (pattern) {
                            case GPP_1:
                                processGPP1(AUTO_STATE.RESET_2);
                                break;
                            case PGP_2:
                                processPGP2(AUTO_STATE.RESET_2);
                                break;
                            case PPG_3:
                                processPPG3(AUTO_STATE.RESET_2);
                                break;
                        }
                        break;

                    }
                    break;
                case RESET_2: //set position for ball 1
                    if(!isPusherUp && stageTimer.milliseconds() > 500){
                        if(sort(2)){
                            stage = 0;
                            stageTimer.reset();
                            autoState = AUTO_STATE.COLLECTION_2;
                            intakeFlag = true;
                        }
                    }
                    break;
                case COLLECTION_2:
                    switch (stage) {
                        case 0: //align with artifacts
                            mecanumCommand.moveToPos(142, -32, -Math.PI / 2); //align with artifacts
                            gate.setPosition(GATE_UP);
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
                            if (stageTimer.milliseconds() > 500) { //replace with whatever time you think is appropriate
                                mecanumCommand.moveToPos(142, -48, -Math.PI / 2); //go to place to intake first artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 3: //set position for second ball
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) { //replace with whatever time you think is appropriate
                                if(sort(0)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 4: //intake second ball
                            if (stageTimer.milliseconds() > 500) { //replace with whatever time you think is appropriate
                                mecanumCommand.moveToPos(142, -63, -Math.PI / 2); //go to place to intake second artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 5: //set position to third ball
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) { //replace with whatever time you think is appropriate
                                if(sort(1)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 6:
                            if (stageTimer.milliseconds() > 500) { //replace with whatever time you think is appropriate
                                stageTimer.reset();
                                intakeFlag = false;
                                stage = 0;
                                gate.setPosition(GATE_DOWN);
                                autoState = AUTO_STATE.FINISH;
                                shooterSubsystem.setMaxRPM(3500);
                                //mecanumCommand.moveToPos(26, -14, 0.5014);
                                mecanumCommand.moveToPos(26, 6, -0.36);
                                hood.setPosition(0.43); //replace with hood position
                                break;
                            }
                            break;

                    }
                    break;
                // 0 purple, 1 green, 2 purple
                case THIRD_SHOT:
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1:
                                processGPP1(AUTO_STATE.FINISH);
                                break;
                            case PGP_2:
                                processPGP2(AUTO_STATE.FINISH);
                                break;
                            case PPG_3:
                                processPPG3(AUTO_STATE.FINISH);

                                break;
                        }
                        break;

                    }
                    break;
                case FINISH:
                    gate.setPosition(GATE_UP);
                    outtakeFlag = false;
                    intakeFlag = false;
                    //mecanumCommand.moveToPos(60, 0, 0); //replace with box position
                    mecanumCommand.stop();
                    break;

            }
        }
    }
    public void processGPP1(AUTO_STATE reset){
        switch (stage) {
            case 0: //turn on outtake
                gate.setPosition(GATE_DOWN);
                outtakeFlag = true;
                stage++;
                stageTimer.reset();
                break;
            case 1: // sort
                if (stageTimer.milliseconds() > 500) {
                    if(sort(0)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 2: //push on
            case 4:
            case 7:
            case 9:
                if (stageTimer.milliseconds() > 200 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 12:
                if (stageTimer.milliseconds() > 400 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 14:
                if (stageTimer.milliseconds() > 200 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;

            case 3: //push off
            case 5:
            case 8:
            case 10:
            case 13:
            case 15:
                if (stageTimer.milliseconds() > 300) {
                    if(halfPush(false)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 6: // sort
                if (stageTimer.milliseconds() > 650) {
                    if(sort(1)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 11: // sort
                if (stageTimer.milliseconds() > 1000 && !isPusherUp) {
                    if(sort(2)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 16:
                stage = 0;
                stageTimer.reset();
                autoState = reset;
                break;
        }
    }
    public void processPGP2(AUTO_STATE reset){
        switch (stage) {
            case 0: //turn on outtake
                gate.setPosition(GATE_DOWN);
                outtakeFlag = true;
                stage++;
                stageTimer.reset();
                break;
            case 1: // sort
                if (stageTimer.milliseconds() > 500) {
                    if(sort(2)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 2: //push on
            case 4:
            case 7:
            case 9:
                if (stageTimer.milliseconds() > 200 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 12:
                if (stageTimer.milliseconds() > 400 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 14:
                if (stageTimer.milliseconds() > 200 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 3: //push off
            case 5:
            case 8:
            case 10:
            case 13:
            case 15:
                if (stageTimer.milliseconds() > 300) {
                    if(halfPush(false)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 6: // sort
                if (stageTimer.milliseconds() > 650) {
                    if(sort()) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 11: // sort
                if (stageTimer.milliseconds() > 1000 && !isPusherUp) {
                    if(sort()) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 16:
                stage = 0;
                stageTimer.reset();
                autoState = reset;
                break;
        }
    }

    public void processPPG3(AUTO_STATE reset){
        switch (stage) {
            case 0: //turn on outtake
                gate.setPosition(GATE_DOWN);
                outtakeFlag = true;
                stage++;
                stageTimer.reset();
                break;
            case 1: // sort
                if (stageTimer.milliseconds() > 500) {
                    if(sort(1)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 2: //push on
            case 4:
            case 7:
            case 9:
                if (stageTimer.milliseconds() > 200 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 12:
                if (stageTimer.milliseconds() > 400 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 14:
                if (stageTimer.milliseconds() > 300 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 3: //push off
            case 5:
            case 8:
            case 10:
            case 13:
            case 15:
                if (stageTimer.milliseconds() > 300) {
                    if(halfPush(false)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 6: // sort
                if (stageTimer.milliseconds() > 650) {
                    if(sort(2)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 11: // sort
                if (stageTimer.milliseconds() > 1000 && !isPusherUp) {
                    if(sort(0)) {
                        sort(0);
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 16:
                stage = 0;
                stageTimer.reset();
                autoState = reset;
                break;
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