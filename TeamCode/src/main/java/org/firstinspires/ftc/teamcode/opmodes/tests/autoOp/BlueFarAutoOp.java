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

@Autonomous (name = "Blue Auto")
public class BlueFarAutoOp extends LinearOpMode {
    //Initialize mecanumCommand, shooterSubsystem, and sorterSubsystem
    private MecanumCommand mecanumCommand;
    private static ShooterSubsystem shooterSubsystem;
    private static SorterSubsystem sorterSubsystem;

    //Initialize a resetTimer
    private ElapsedTime resetTimer;

    //Initialize LogitechVisionSubsystem
    private LogitechVisionSubsystem logitechVisionSubsystem;

    //Initialize target X
    private double targetX = Double.NaN; // X position of alliance-specific tag

    //Create an enum to power the state machine
    enum AUTO_STATE {
        FIRST_SHOT, RESET, COLLECTION_1, SECOND_SHOT, RESET_2, COLLECTION_2, THIRD_SHOT, FINISH
    }

    //Create an enum for each possible pattern (sorted auto)
    enum PATTERN {
        GPP_1, //Tag ID 21
        PGP_2, //Tag ID 22
        PPG_3 //Tag ID 23
    }

    //Pusher variables
    private static final double PUSHER_UP = 0.0;
    private static final double PUSHER_DOWN = 1.0;
    private static final long PUSHER_TIME = 150;
    private static boolean isPusherUp = false;
    private static final ElapsedTime pusherTimer = new ElapsedTime();

    //Stage timer, to time the changes between states
    private static final ElapsedTime stageTimer = new ElapsedTime();

    //Sorter variables
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    private static double pos1 = 0.085;
    private static double pos2 = 0.515;
    private static double pos3 = 0.96;
    private static int standardms = 1000;

    private static final long SORTER_TIME = 250;
    private static int currentSort = -1;

    //Intake timers
    private static final ElapsedTime intakeTimer = new ElapsedTime();
    private static final long INTAKE_WAIT = 700;
    private static boolean intakeWasOn = false;

    //Collected Count
    private static int collectedCount = 0;

    //Hood position variable
    private static double hoodPos = 0.359;

    //Gate variables
    private static final double GATE_UP = 1.0;
    private static final double GATE_DOWN = 0.675;
//
//    private double initialHeading = 0.0;
//    private int initialTurretPos = 0;
//    private static final double TICKS_PER_RAD = 100.0;

    //Initialize motors and servos
    private static DcMotor shooter;
    private static Servo pusher;
    private static Servo hood;
    private static Servo sorter;
    private static Servo gate;
    private static DcMotorEx intake;
    private static DcMotorEx turret;

    //Outtake and intake states
    boolean outtakeFlag = false;
    boolean intakeFlag = false;

    //Set initial autoState to First Shot
    AUTO_STATE autoState = AUTO_STATE.FIRST_SHOT;

    //Stage variable for switch-case statements within the state machine
    private static int stage;

    //Initial pattern is PPG_3 (must be set to something in case AprilTag detection fails)
    PATTERN pattern = PATTERN.PPG_3; // default

    //halfPush, taking in boolean isUp, which either sets pusher to up or down
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

    //sort, taking in integer sp, which sets the position to the specified position
    static boolean sort(int sp) {
        double pos = (sp == 0) ? pos1 : (sp == 1) ? pos2 : pos3;
        sorter.setPosition(pos);
        currentSort = sp;
        sorterTimer.reset();
        return true;
    }

    //sort with no arguments, which just sorts to the next position using manualSpin
    static boolean sort() {
        if (sorterTimer.milliseconds() > 500) {
            sorterTimer.reset();
            sorterSubsystem.manualSpin();
            return true;
        }
        return false;
    }

    //shoot, taking in boolean isOn, which turns the shooter on or off
    static void shoot(boolean isOn) {
        if (isOn) {
            shooterSubsystem.spinup();
        } else {
            shooterSubsystem.stopShooter();
        }
    }

    //intake, taking in boolean isOn, which turns the intake on or off
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

        //initialize mecanumCommand, shooterSubsystem, sorterSubsystem, and resetTimer
        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "");
        resetTimer = new ElapsedTime();

        //bind the shooter, pusher, sorter, hood, gate, intake, and turret to their respective motors and servos
        shooter = hw.shooter;
        pusher = hw.pusher;
        sorter = hw.sorter;
        hood = hw.hood;
        gate = hw.gate;
        intake = hw.intake;
        turret = hw.llmotor;

        //Reset encoder of turret, set target position to 0, and run to that position
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set the sorter, pusher, hood, and gate initial positions
        sorter.setPosition(pos1);
        pusher.setPosition(PUSHER_DOWN);
        hood.setPosition(hoodPos);
        gate.setPosition(GATE_DOWN);

        //set position and stage to 0
        int position = 0;
        stage = 0;

        //initialize logitechVisionSubsystem
        logitechVisionSubsystem = new LogitechVisionSubsystem(hw, "BLUE");

        //update telemetry
        telemetry.update();

        long scanStart = System.currentTimeMillis();
        long scanTimeout = 5000;

        // Detect obelisk pattern, and set pattern based on that
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

            //Then update telemetry with this data, in order to know for sure
            telemetry.addData("Detected Obelisk", detected);
            telemetry.addData("Pattern", pattern);
            telemetry.addData("Target X", targetX);
            telemetry.update();

        }

        //Wait for start
        waitForStart();

        //While running
        while (opModeIsActive()) {
            //Call motorProcess and processOdometry
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            //Set turret power to be on, and set shoot and intake to outtake and intakeFlag
            turret.setPower(1.0);
            shoot(outtakeFlag);
            intake(intakeFlag);

            //Update telemetry with stage, pattern, position, sorter and stageTimer
            telemetry.addData("stage", stage);
            telemetry.addData("Pattern", pattern);
            telemetry.addData("position: ", position);
            telemetry.addData("sorterTimer: ", sorterTimer.milliseconds());
            telemetry.addData("stageTimer: ", stageTimer.milliseconds());

            //Process Telemetry
            processTelemetry();

            //State machine, going through the enum autoState
            switch (autoState) {
                case FIRST_SHOT:
                    //Set max RPM to 3500 rpm, move to initial position, and set hood position
                    shooterSubsystem.setMaxRPM(3500);
                    mecanumCommand.moveToPos(26, -6, 0.36);
                    hood.setPosition(0.43);

                    //Depending on pattern, call respective processPattern function
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
                //Set sorter position and then turn on intake
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
                //Align with the artifacts, put gate up, intake them, sort, move forward, and repeat the last two steps until sorter is full
                case COLLECTION_1:
                    switch (stage) {
                        case 0: //align with artifacts
                            mecanumCommand.moveToPos(82, 32, Math.PI / 2); //align with artifacts
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
                            if (stageTimer.milliseconds() > 500) {
                                mecanumCommand.moveToPos(82, 48, Math.PI / 2); //go to place to intake first artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 3: //set position for second ball
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if(sort(1)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 4: //intake second ball
                            if (stageTimer.milliseconds() > 750) {
                                mecanumCommand.moveToPos(82, 63, Math.PI / 2); //go to place to intake second artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 5: //set position to third ball
                            if (stageTimer.milliseconds() > 750 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if(sort(2)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 6:
                            if (stageTimer.milliseconds() > 750) {
                                stageTimer.reset();
                                stage = 0;
                                autoState = AUTO_STATE.SECOND_SHOT;
                                shooterSubsystem.setMaxRPM(3500);
                                mecanumCommand.moveToPos(26, -6, 0.355);
                                hood.setPosition(0.43);
                                break;
                            }
                            break;

                    }
                    break;
                //Repeat last process once more
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
                            mecanumCommand.moveToPos(142, 28, Math.PI / 2); //align with artifacts
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
                            if (stageTimer.milliseconds() > 500) {
                                mecanumCommand.moveToPos(142, 48, Math.PI / 2); //go to place to intake first artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 3: //set position for second ball
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if(sort(0)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 4: //intake second ball
                            if (stageTimer.milliseconds() > 500) {
                                mecanumCommand.moveToPos(142, 63, Math.PI / 2); //go to place to intake second artifact
                                stageTimer.reset();
                                stage++;
                            }
                            break;
                        case 5: //set position to third ball
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if(sort(1)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            }
                            break;
                        case 6:
                            if (stageTimer.milliseconds() > 750) {
                                stageTimer.reset();
                                stage = 0;
                                autoState = AUTO_STATE.FINISH;
                                shooterSubsystem.setMaxRPM(3500);
                                //mecanumCommand.moveToPos(26, -14, 0.5014);
                                mecanumCommand.moveToPos(26, -6, 0.355);
                                hood.setPosition(0.43);
                                break;
                            }
                            break;

                    }
                    break;
                //Shoot once more
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
                //Put gate up, turn off outtake and intake
                case FINISH:
                    gate.setPosition(GATE_UP);
                    outtakeFlag = false;
                    intakeFlag = false;

                    mecanumCommand.stop();
                    break;

            }
        }
    }
    //processGPP1, takes in an AUTO_STATE to navigate to after the process, and then unloads and shoots all artifacts in GPP order
    public void processGPP1(AUTO_STATE reset){
        switch (stage) {
            case 0: //turn on outtake
                intakeFlag = false;
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
    //processPGP2, takes in an AUTO_STATE to navigate to after the process, and then unloads and shoots all artifacts in PGP order
    public void processPGP2(AUTO_STATE reset){
        switch (stage) {
            case 0: //turn on outtake
                intakeFlag = false;
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
    //processPPG3, takes in an AUTO_STATE to navigate to after the process, and then unloads and shoots all artifacts in PPG order
    public void processPPG3(AUTO_STATE reset){
        switch (stage) {
            case 0: //turn on outtake
                intakeFlag = false;
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

    //processTelemetry(), updates telemetry with x, y, theta, and shooter velocity
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