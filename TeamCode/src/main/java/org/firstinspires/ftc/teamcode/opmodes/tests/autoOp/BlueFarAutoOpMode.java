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

@Autonomous (name = "Blue Auto")
public class BlueFarAutoOpMode extends LinearOpMode {
    private static MecanumCommand mecanumCommand;
    private static ShooterSubsystem shooterSubsystem;
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
    private static final long PUSHER_TIME = 100;
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

    private static DcMotor shooter;
    private static Servo pusher;
    private static Servo hood;
    private static Servo sorter;
    private static DcMotorEx intake;

    private static boolean previousPushState = false;
    private static boolean currentPushState;

    private static boolean outtakeFlag = false;
    private static boolean intakeFlag = false;

    private static int stage;
    PATTERN pattern = PATTERN.PPG_3; // default
    static AUTO_STATE autoState = AUTO_STATE.FIRST_SHOT;


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
        if (!isPusherUp && pusherTimer.milliseconds() >= PUSHER_TIME) {
            if (sp != currentSort) {
                double pos = (sp == 0) ? pos1 : (sp == 1) ? pos2 : pos3;
                sorter.setPosition(pos);
                currentSort = sp;
                sorterTimer.reset();
                return false; // need to wait SORTER_TIME before reporting success
            } else {
                if (sorterTimer.milliseconds() >= SORTER_TIME) {
                    return true;
                }
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


    static void setup(double x, double y, double theta, double rpm, double hoodAngle, boolean intakeOn, boolean outtakeOn){
        intakeFlag = intakeOn;
        outtakeFlag = outtakeOn;
        shooterSubsystem.setMaxRPM(rpm);
        mecanumCommand.moveToPos(x, y, theta);
        hood.setPosition(hoodAngle);
    }

    static void shootInOrder(int pos1, int pos2, int pos3, AUTO_STATE state){
        switch (stage) {
            case 0: //turn on outtake
                outtakeFlag = true;
                stage++;
                stageTimer.reset();
                break;
            case 1: // sort
                if (stageTimer.milliseconds() > 500) {
                    if(sort(pos1)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 2: //push on
            case 5:
            case 8:
                if (stageTimer.milliseconds() > 100 && shooterSubsystem.isRPMReached()) {
                    halfPush(true);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 3: //push off
            case 6:
            case 9:
                if (stageTimer.milliseconds() > 400) {
                    halfPush(false);
                    stage++;
                    stageTimer.reset();
                }
                break;
            case 4: // sort
                if (stageTimer.milliseconds() > 500) {
                    if(sort(pos2)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 7: // sort
                if (stageTimer.milliseconds() > 750 && !isPusherUp) {
                    if(sort(pos3)) {
                        stage++;
                        stageTimer.reset();
                    }
                }
                break;
            case 10:
                stage = 0;
                stageTimer.reset();
                autoState = state;
                break;
        }
    }

    static void collection(double x, double startY, double theta, int pos2, int pos3, AUTO_STATE state){
        switch (stage) {
            case 0: //align with artifacts
                mecanumCommand.moveToPos(x, startY, theta); //align with artifacts
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
                    mecanumCommand.moveToPos(x, startY + 12, theta); //go to place to intake first artifact
                    stageTimer.reset();
                    stage++;
                }
                break;
            case 3: //set position for second ball
                if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) { //replace with whatever time you think is appropriate
                    if(sort(pos2)) {
                        stageTimer.reset();
                        stage++;
                    }
                }
                break;
            case 4: //intake second ball
                if (stageTimer.milliseconds() > 500) { //replace with whatever time you think is appropriate
                    mecanumCommand.moveToPos(x, startY + 31, theta); //go to place to intake second artifact
                    stageTimer.reset();
                    stage++;
                }
                break;
            case 5: //set position to third ball
                if (stageTimer.milliseconds() > 250 && intakeTimer.milliseconds() >= INTAKE_WAIT) { //replace with whatever time you think is appropriate
                    if(sort(pos3)) {
                        stageTimer.reset();
                        stage++;
                    }
                }
                break;
            case 6:
                if (stageTimer.milliseconds() > 500) { //replace with whatever time you think is appropriate
                    stageTimer.reset();
                    stage = 0;
                    autoState = state;
                    setup(26, -6, 0.4014, 3800, 0.43, true, false);
                    break;
                }
                break;

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
                    setup(26, -6, 0.4014, 3800, 0.43, true, false);
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1:
                                shootInOrder(0, 1, 2, AUTO_STATE.RESET);
                                break;
                            case PGP_2:
                                shootInOrder(2, 0, 1, AUTO_STATE.RESET);
                                break;
                            case PPG_3:
                                shootInOrder(1, 2, 0, AUTO_STATE.RESET);
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
                    collection(80, 32, Math.PI/2, 1, 2, AUTO_STATE.SECOND_SHOT);
                    break;

                case SECOND_SHOT:
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1:
                                shootInOrder(0, 1, 2, AUTO_STATE.RESET_2);
                                break;
                            case PGP_2:
                                shootInOrder(2, 0, 1, AUTO_STATE.RESET_2);
                                break;
                            case PPG_3:
                                shootInOrder(1, 2, 0, AUTO_STATE.RESET_2);
                                break;
                        }
                        break;
                    }
                    break;
                case RESET_2: //set position for ball 1
                    if(!isPusherUp && stageTimer.milliseconds() > 500){
                        if(sort(0)){
                            stageTimer.reset();
                            autoState = AUTO_STATE.COLLECTION_2;
                        }
                    }
                    break;
                case COLLECTION_2:
                    collection(80, 32, Math.PI/2, 1, 2, AUTO_STATE.THIRD_SHOT);
                    break;
                // 0 purple, 1 green, 2 purple
                case THIRD_SHOT:
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1:
                                shootInOrder(1, 2, 0, AUTO_STATE.FINISH);
                                break;
                            case PGP_2:
                                shootInOrder(0, 1, 2, AUTO_STATE.FINISH);
                                break;
                            case PPG_3:
                                shootInOrder(2, 0, 1, AUTO_STATE.FINISH);
                                break;
                        }
                        break;
                    }
                    break;
                case FINISH:
                    setup(60, 0, 0, 3800, 0.43, false, false);
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
        telemetry.addData("Stage: ", stage);
        telemetry.update();
    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
