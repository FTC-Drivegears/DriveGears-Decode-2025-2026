package org.firstinspires.ftc.teamcode.opmodes.tests.autoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.opmodes.tests.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismAuto;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.FieldOrientedOffset;
import org.firstinspires.ftc.teamcode.util.PusherConsts;


@Autonomous()
public class CloseRedAutoOp extends LinearOpMode {

    private MecanumCommand mecanumCommand;
    private static ShooterSubsystem shooterSubsystem;
    private static SorterSubsystem sorterSubsystem;
    private ElapsedTime resetTimer;
    private LimelightVision LimelightVision;
    private Limelight3A limelight;
    private LLResult llResult;

    enum AUTO_STATE {
        FIRST_SHOT, RESET, COLLECTION_1, SECOND_SHOT, RESET_2, COLLECTION_2, THIRD_SHOT, FINISH
    }

    enum PATTERN {
        GPP_1, PGP_2, PPG_3
    }

    // Pusher positions
    private static final double PUSHER_UP_L      = PusherConsts.PUSHER_UP_POSITION_L + 0.15;
    private static final double PUSHER_DOWN_L     = PusherConsts.PUSHER_DOWN_POSITION_L;
    private static final double PUSHER_UP_R      = PusherConsts.PUSHER_UP_POSITION_R - 0.15;
    private static final double PUSHER_DOWN_R     = PusherConsts.PUSHER_DOWN_POSITION_R;

    // 2/3 pre-load position — same as TeleOp, holds ball against flywheel
    // while waiting for RPM to reach target, then fires all the way
    private static final double PUSHER_PARTIAL_L  =
            PUSHER_DOWN_L + (PUSHER_UP_L - PUSHER_DOWN_L) * (2.0 / 3.0);
    private static final double PUSHER_PARTIAL_R  =
            PUSHER_DOWN_R + (PUSHER_UP_R - PUSHER_DOWN_R) * (2.0 / 3.0);

    private static final long   PUSHER_TIME        = 100;
    private static boolean      isPusherUp         = false;
    private static final ElapsedTime pusherTimer   = new ElapsedTime();
    private static final long   PUSHER_SAFE_MARGIN = 150;

    private static final ElapsedTime stageTimer  = new ElapsedTime();
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    // SORTER POS
    private static double pos1      = 0.09;
    private static double pos2      = 0.44;
    private static double pos3      = 0.82;
    private static int    standardms = 1000;

    private static final long SORTER_TIME = 250;
    private static int currentSort = -1;

    private static final ElapsedTime intakeTimer = new ElapsedTime();
    private static final long   INTAKE_WAIT  = 700;
    private static boolean      intakeWasOn  = false;

    private static DcMotor   shooter;
    private static Servo     pusher_L;
    private static Servo     pusher_R;
    private static Servo     hood;
    private static Servo     sorter;
    private static DcMotorEx intake;
    private TurretMechanismAuto turret;

    boolean outtakeFlag = false;
    boolean intakeFlag  = false;

    private boolean shooterAtSpeed() {

        return shooterSubsystem.isRPMReached();
    }



    AUTO_STATE autoState = AUTO_STATE.FIRST_SHOT;
    private static int stage;

    PATTERN pattern = PATTERN.PPG_3;

    static boolean pusherReady() {
        return !isPusherUp && pusherTimer.milliseconds() >= (PUSHER_TIME + PUSHER_SAFE_MARGIN);
    }

    /**
     * Holds pusher at 2/3 pre-load position while shooter spins up.
     * Identical to TeleOp: ball sits against flywheel ready to fire.
     */
    static void preload() {
        if (!isPusherUp) {
            pusher_L.setPosition(PUSHER_PARTIAL_L);
            pusher_R.setPosition(PUSHER_PARTIAL_R);
        }
    }

    /** Moves pusher to full up (fire) or full down. Returns true when move is complete. */
    static boolean halfPush(boolean isUp) {

        if (isUp) {

            if (!isPusherUp) {
                pusher_L.setPosition(PUSHER_UP_L);
                pusher_R.setPosition(PUSHER_UP_R);
                isPusherUp = true;
                pusherTimer.reset();
            }

            return pusherTimer.milliseconds() >= PUSHER_TIME;
        }

        else {

            if (isPusherUp) {
                pusher_L.setPosition(PUSHER_DOWN_L);
                pusher_R.setPosition(PUSHER_DOWN_R);
                isPusherUp = false;
                pusherTimer.reset();
            }

            return pusherTimer.milliseconds() >= PUSHER_TIME;
        }
    }

    static boolean sort(int sp) {
        if (!pusherReady()) return false;
        double pos = (sp == 0) ? pos1 : (sp == 1) ? pos2 : pos3;
        sorter.setPosition(pos);
        currentSort = sp;
        sorterTimer.reset();
        return true;
    }

    static boolean sort() {
        if (!pusherReady()) return false;
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
            if (!intakeWasOn) { intakeTimer.reset(); intakeWasOn = true; }
        } else {
            intake.setPower(0.0);
            intakeWasOn = false;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware hw = Hardware.getInstance(hardwareMap);
        FieldOrientedOffset.headingOffsetRad = Math.PI / 2;

        mecanumCommand   = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        sorterSubsystem  = new SorterSubsystem(hw, shooterSubsystem, this, telemetry, "");
        resetTimer       = new ElapsedTime();

        shooter  = hw.shooter;
        pusher_L = hw.pusher_L;
        pusher_R = hw.pusher_R;
        sorter   = hw.sorter;
        hood     = hw.hood;
        intake   = hw.intake;

        turret = new TurretMechanismAuto();
        turret.init(hardwareMap);
        // No setkP/setkD override — uses our tuned defaults (kP=0.050, kD=0.009)
        turret.setTarget(0, 3300, 0.1);

        limelight = hw.limelight;
        limelight.pipelineSwitch(0);
        limelight.start();

        sorter.setPosition(pos1);
        pusher_L.setPosition(PUSHER_DOWN_L);
        pusher_R.setPosition(PUSHER_DOWN_R);

        int position = 0;
        stage = 0;

        LimelightVision = new LimelightVision(hw);
        telemetry.update();

        // Detect obelisk pattern before start
        while (!isStarted() && !isStopRequested()) {
            String detected = LimelightVision.pattern();
            if (detected != null && !detected.equals("UNKNOWN")) {
                switch (detected) {
                    case "GPP": pattern = PATTERN.GPP_1; break;
                    case "PGP": pattern = PATTERN.PGP_2; break;
                    case "PPG": pattern = PATTERN.PPG_3; break;
                }
            }

            llResult = limelight.getLatestResult();
            Double tx = null, ty = null;
            if (llResult != null && llResult.isValid()) {
                telemetry.addData("Tag Detected", "ID: " + llResult.getFiducialResults().get(0).getFiducialId());
                tx = llResult.getTx();
                ty = llResult.getTy();
            } else {
                telemetry.addData("Tag Detected", "None");
            }
            telemetry.addData("Pattern:", detected);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.update();
        }

        limelight.pipelineSwitch(8);

        waitForStart();

        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            shoot(outtakeFlag);
            intake(intakeFlag);

            llResult = limelight.getLatestResult();
            Double tx = null, ty = null;
            if (llResult != null && llResult.isValid()) {
                tx = llResult.getTx();
                ty = llResult.getTy();
            }

            // Full update signature — passes odo position for accurate world-angle tracking
            long llStaleMs = (llResult != null) ? llResult.getStaleness() : -1;
            turret.update();   // autoAimOn always true in auto

            telemetry.addData("stage",      stage);
            telemetry.addData("Pattern",    pattern);
            telemetry.addData("tx",         tx);
            telemetry.addData("ty",         ty);
            telemetry.addData("RPM ready",  shooterSubsystem.isRPMReached());
            processTelemetry();

            switch (autoState) {
                case FIRST_SHOT:
                    outtakeFlag = true;
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    mecanumCommand.moveToPos(-127, 0, -Math.PI/4);
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1: processGPP1(AUTO_STATE.RESET);  break;
                            case PGP_2: processPGP2(AUTO_STATE.RESET);  break;
                            case PPG_3: processPPG3(AUTO_STATE.RESET);  break;
                        }
                    }
                    break;

                case RESET:
                    if (!isPusherUp && stageTimer.milliseconds() > 500) {
                        if (sort(0)) {
                            stageTimer.reset();
                            autoState = AUTO_STATE.COLLECTION_1;
                            stage = 0;
                            intakeFlag = true;
                        }
                    }
                    break;

                case COLLECTION_1:
                    switch (stage) {
                        case 0:
//                            mecanumCommand.moveToPos(0, 0, 0);
                            mecanumCommand.moveToPos(76, -30, -Math.PI / 2);
                            stageTimer.reset();
                            stage++;
                            break;
                        case 1:
                            if (mecanumCommand.isPositionReached()) {
                                stage++;
                                stageTimer.reset();
                            } break;
                        case 2:
                            if (stageTimer.milliseconds() > 250) {
//                                mecanumCommand.moveToPos(0, 0, 0);
                                mecanumCommand.moveToPos(76, -46, -Math.PI / 2);
                                stageTimer.reset(); stage++;
                            } break;
                        case 3:
                            if (stageTimer.milliseconds() > 350 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if (sort(1)) {
                                    stageTimer.reset();
                                    stage++;
                                }
                            } break;
                        case 4:
                            if (stageTimer.milliseconds() > 500) {
//                                mecanumCommand.moveToPos(0, 0, 0);
                                mecanumCommand.moveToPos(76, -60, -Math.PI / 2);
                                stageTimer.reset();
                                stage++;
                            } break;
                        case 5:
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if (sort(2)) { stageTimer.reset(); stage++; }
                            } break;
                        case 6:
                            if (stageTimer.milliseconds() > 500) {
                                stageTimer.reset(); stage = 0;
                                autoState = AUTO_STATE.SECOND_SHOT;
//                                mecanumCommand.moveToPos(0, 0, 0);
                                mecanumCommand.moveToPos(26, 6, -0.43);
                            } break;
                    }
                    break;

                case SECOND_SHOT:
                    outtakeFlag = true;
                    shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                    if (mecanumCommand.isPositionReached()) {
                        intakeFlag = false;
                        switch (pattern) {
                            case GPP_1: processGPP1(AUTO_STATE.RESET_2); break;
                            case PGP_2: processPGP2(AUTO_STATE.RESET_2); break;
                            case PPG_3: processPPG3(AUTO_STATE.RESET_2); break;
                        }
                    }
                    break;

                case RESET_2:
                    if (!isPusherUp && stageTimer.milliseconds() > 500) {
                        if (sort(2)) {
                            stage = -1; stageTimer.reset();
                            autoState = AUTO_STATE.COLLECTION_2;
                            intakeFlag = true;
                        }
                    }
                    break;

                case COLLECTION_2:
                    switch (stage) {
                        case -1:
                            mecanumCommand.moveToPos(136, -20, -Math.PI / 2);
                            stage++;
                            stageTimer.reset();
                            break;
                        case 0:
                            if(stageTimer.milliseconds() > 500){
                                mecanumCommand.moveToPos(136, -38, -Math.PI / 2);
                                stageTimer.reset(); stage++; break;
                            };
//                            mecanumCommand.moveToPos(0, 0, 0);
                        case 1:
                            if (mecanumCommand.isPositionReached()) { stage++; stageTimer.reset(); } break;
                        case 2:
                            if (stageTimer.milliseconds() > 500) {
                                mecanumCommand.moveToPos(136, -52, -Math.PI / 2);
                                stageTimer.reset(); stage++;
                            } break;
                        case 3:
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if (sort(0)) { stageTimer.reset(); stage++; }
                            } break;
                        case 4:
                            if (stageTimer.milliseconds() > 500) {
                                mecanumCommand.moveToPos(136, -68, -Math.PI / 2);
                                stageTimer.reset(); stage++;
                            } break;
                        case 5:
                            if (stageTimer.milliseconds() > 500 && intakeTimer.milliseconds() >= INTAKE_WAIT) {
                                if (sort(1)) { stageTimer.reset(); stage++; }
                            } break;
                        case 6:
                            if (stageTimer.milliseconds() > 750) {
                                stageTimer.reset(); stage = 0;
                                autoState = AUTO_STATE.THIRD_SHOT;
                                //shooterSubsystem.setMaxRPM((int) Math.round(turret.getShootRPM()));
                                mecanumCommand.moveToPos(26, 6, -0.43);
                            } break;
                    }
                    break;

                case THIRD_SHOT:
                    if (mecanumCommand.isPositionReached()) {
                        switch (pattern) {
                            case GPP_1: processGPP1(AUTO_STATE.FINISH); break;
                            case PGP_2: processPGP2(AUTO_STATE.FINISH); break;
                            case PPG_3: processPPG3(AUTO_STATE.FINISH); break;
                        }
                    }
                    break;

                case FINISH:
                    switch(stage){
                        case 0:
                            mecanumCommand.moveToPos(100, 0, 0);
                            stageTimer.reset();
                            stage++;
                            break;
                        case 1:
                            outtakeFlag = false;
                            intakeFlag  = false;
                            mecanumCommand.stop();
                            break;
                    }

            }
        }

    }

    // -------------------------------------------------------------------------
    // Shooting sequences — pusher pre-loads at 2/3 while waiting for RPM,
    // fires all the way once shooterAtSpeed() is true. Matches TeleOp behaviour.
    // -------------------------------------------------------------------------

    public void processGPP1(AUTO_STATE reset) {
        switch (stage) {
            case 0:
                intakeFlag = false; outtakeFlag = true;
                stage++; stageTimer.reset(); break;
            case 1: //500
                if (sort(0)) {
                    if (stageTimer.milliseconds() > 250) { stage++; stageTimer.reset(); }
                } break;
            case 2:
                if(shooterAtSpeed()  && halfPush(true)) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 5:
                // Pre-load at 2/3 while waiting; fire all the way when ready//600
                if(shooterAtSpeed()  && halfPush(true)) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 8:
                if(shooterAtSpeed()  && halfPush(true) && stageTimer.milliseconds() > 200) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 3: case 6: case 9:
                if (stageTimer.milliseconds() > 200) { //300
                    if (halfPush(false)) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 4:
                if (sort(1)) { //650
                    if (stageTimer.milliseconds() > 300) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 7:
                if (sort(2)) { //1000
                    if (stageTimer.milliseconds() > 250 ) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 10:
                stage = 0; stageTimer.reset();
                autoState = reset;
                break;
        }
    }

    public void processPGP2(AUTO_STATE reset) {
        switch (stage) {
            case 0:
                intakeFlag = false; outtakeFlag = true;
                stage++; stageTimer.reset(); break;
            case 1: //500
                if (sort(2)) {
                    if (stageTimer.milliseconds() > 250) { stage++; stageTimer.reset(); }
                } break;
            case 2:
                if(shooterAtSpeed()  && halfPush(true)) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 5:
                // Pre-load at 2/3 while waiting; fire all the way when ready//600
                if(shooterAtSpeed()  && halfPush(true)) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 8:
                if(shooterAtSpeed()  && halfPush(true) && stageTimer.milliseconds() > 200) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 3: case 6: case 9:
                if (stageTimer.milliseconds() > 225) { //300
                    if (halfPush(false)) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 4:
                if (sort(0)) { //650
                    if (stageTimer.milliseconds() > 400 ) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 7:
                if (sort(1)) { //1000
                    if (stageTimer.milliseconds() > 250) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 10:
                stage = 0; stageTimer.reset();
                autoState = reset;
                break;
        }
    }

    public void processPPG3(AUTO_STATE reset) {
        switch (stage) {
            case 0:
                intakeFlag = false; outtakeFlag = true;
                stage++; stageTimer.reset(); break;
            case 1: //500
                if (sort(1)) {
                    if (stageTimer.milliseconds() > 250) { stage++; stageTimer.reset(); }
                } break;
            case 2:
                if(shooterAtSpeed()  && halfPush(true)) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 5:
                // Pre-load at 2/3 while waiting; fire all the way when ready//600
                if(shooterAtSpeed()  && halfPush(true)) {
                    stage++;
                    stageTimer.reset();
                } break;

            case 8:
                if(shooterAtSpeed()  && stageTimer.milliseconds() > 200) {
                    if(halfPush(true)) {
                        stage++;
                        stageTimer.reset();
                    }
                } break;

            case 3: case 6: case 9:
                if (stageTimer.milliseconds() > 200) { //300
                    if (halfPush(false)) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 4:
                if (sort(2)) { //650
                    if (stageTimer.milliseconds() > 250) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 7:
                if (sort(0)) { //1000
                    if (stageTimer.milliseconds() > 300) {
                        stage++;
                        stageTimer.reset(); }
                } break;
            case 10:
                stage = 0; stageTimer.reset();
                autoState = reset;
                break;
        }
    }

    public void processTelemetry() {
        telemetry.addData("resetTimer", resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X",        mecanumCommand.getX());
        telemetry.addData("Y",        mecanumCommand.getY());
        telemetry.addData("Theta",    mecanumCommand.getOdoHeading());
        telemetry.addData("Shooter",  shooterSubsystem.getShooterVelocity());
        telemetry.addData("Stage",    stage);
        telemetry.addData("StageTimer", stageTimer.milliseconds());
        telemetry.update();
    }
}