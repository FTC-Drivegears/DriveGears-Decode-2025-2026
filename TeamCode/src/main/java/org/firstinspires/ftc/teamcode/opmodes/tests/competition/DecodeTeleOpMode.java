package org.firstinspires.ftc.teamcode.opmodes.tests.competition;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.opmodes.tests.vision.LogitechVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.PusherConsts;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class DecodeTeleOpMode extends LinearOpMode {

    //Define mecanumCommand  and hardware
    private MecanumCommand mecanumCommand;
    private Hardware hw;

    //Define variabls for turret turning
    private double theta;
    final double TURN_GAIN = 0.5;
    final double MAX_AUTO_TURN = 0.3; //max speed
    private double previousTurn = 0; //last turn value
    //final double goal = 0;

    //Define servos, motors, and limelight
    private DcMotor intake;
    private DcMotor shooter;
    private Servo pusher;
    private Servo hood;
    private Servo light;
    private Servo gate;
    private DcMotorEx llmotor;
    private Limelight3A limelight;

    // Create sorter subsystem and shooter subsystem
    private SorterSubsystem sorterSubsystem;
    private ShooterSubsystem shooterSubsystem;

    //Create ElapsedTime variables for sorter, outtake, pusher, and color sensor
    private final ElapsedTime sorterTimer = new ElapsedTime();

    private final ElapsedTime outtakeTimer = new ElapsedTime();

    private final ElapsedTime pusherTimer = new ElapsedTime();

    private final ElapsedTime colorSensingTimer = new ElapsedTime();

    //Shooter Presets
    private final double FAR_HOOD = 0.4;
    private final int FAR_SHOOT_SPEED = 3700;
    private final double MID_HOOD = 0.6;
    private final int MID_SHOOT_SPEED = 3050;
    private final double CLOSE_HOOD = 0.846;
    private final int CLOSE_SHOOT_SPEED = 2500;

    //Gate positions

    private static final double GATE_UP = 1.0;
    private static final double GATE_DOWN = 0.65;

    //Initialize Logitech Vision Subsystem
    private LogitechVisionSubsystem vision;

    //enum for drive type, containing both robot and field oriented movement types
    private enum DRIVETYPE {
        ROBOTORIENTED, FIELDORIENTED
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //state variables for x and y buttons (flywheel and pusher buttons)
        boolean previousXState = false;
        boolean previousYState = false;

        boolean currentXState;
        boolean currentYState;

        //booleans for intaking
        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean rightTriggerPressed = false;
        boolean leftTriggerPressed = false;

        //variables for hood position and shooter speed
        double hoodPos = 0.846;
        double shootSpeed = 4000;

        //setting inital drive type to Robot-oriented move
        DRIVETYPE drivetype = DRIVETYPE.ROBOTORIENTED;

        //initialize hardwareMap
        hw = Hardware.getInstance(hardwareMap);

        //initialize mecanumCommmand, shooterSubsystem, and logitechVisionSystem
        mecanumCommand = new MecanumCommand(hw);
        shooterSubsystem = new ShooterSubsystem(hw);
        vision = new LogitechVisionSubsystem(hw, "RED");

        //define pusher, light, and set their positions
        pusher = hw.pusher;
        light = hw.light;
        pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
        hw.light.setPosition(0.0);

        //set positions of sorter, gate. and hood
        hw.sorter.setPosition(0.085);
        hw.gate.setPosition(GATE_DOWN);
        hw.hood.setPosition(hoodPos);

        //define intake, shooter, hood, and gate
        intake = hw.intake;
        shooter = hw.shooter;
        hood = hw.hood;
        gate = hw.gate;

        //initialize limelight and llmotor
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llmotor = hardwareMap.get(DcMotorEx.class, "llmotor");
        limelight.pipelineSwitch(7);
        limelight.start();

        //Start telemetry, and add line 'waiting for start'
        telemetry.addLine("waiting for start");
        telemetry.update();

        //Set intake direction to reverse
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //create sorterSubsystem
        if (sorterSubsystem == null) {
            sorterSubsystem = new SorterSubsystem(hw, this, telemetry, "");
        }

        //while opMode is in initialize mode, if a is pressed, set mode to robot-oriented movement
        //otherwise, set to field-oriented movement
        while (opModeInInit()) {
            if (gamepad1.a) {
                drivetype = DRIVETYPE.ROBOTORIENTED;
            }

            if (gamepad1.y) {
                drivetype = DRIVETYPE.FIELDORIENTED;
            }
            telemetry.update();
        }

        //wait for start
        waitForStart();

        while (opModeIsActive()) {

            Double headingError = vision.getTargetYaw();
            boolean targetFound = (headingError != null);

            double turn = 0;

            mecanumCommand.processOdometry();

            //sets movement type to field or robot-oriented
          if (drivetype == DRIVETYPE.FIELDORIENTED) {
              theta = mecanumCommand.fieldOrientedMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
          }
            if (drivetype == DRIVETYPE.ROBOTORIENTED) {
                theta = mecanumCommand.normalMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            //intake mechanism
            if (gamepad1.right_trigger > 0) {
                //if right trigger wasn't pressed, and it is pressed, then turn intake motor on
                if (!rightTriggerPressed) {
                    rightTriggerPressed = true;
                    isIntakeMotorOn = !isIntakeMotorOn;
                    if (isIntakeMotorOn) {
                        intake.setPower(0.8);
                        //also set gate to be up
                        gate.setPosition(GATE_UP);
                    //otherwise, set intake power to zero, and put gate back down
                    } else {
                        intake.setPower(0);
                        gate.setPosition(GATE_DOWN);
                    }
                }
            } else
                rightTriggerPressed = false;

            //repeat process for left trigger, except have the intake motor move the opposite way
            if (gamepad1.left_trigger > 0) {
                if (!leftTriggerPressed) {
                    leftTriggerPressed = true;
                    isIntakeMotorOn = !isIntakeMotorOn;
                    if (isIntakeMotorOn) {
                        intake.setPower(-0.8);
                        gate.setPosition(GATE_UP);
                    } else {
                        intake.setPower(0);
                        gate.setPosition(GATE_DOWN);
                    }
                }
            } else {
                leftTriggerPressed = false;
            }

            //if b button is pressed and 500ms have passed since the last sort, manually spin sorter
            if (gamepad1.b && sorterTimer.milliseconds() > 500) {
                sorterTimer.reset();
                sorterSubsystem.manualSpin();
            }


            //if y button is pressed, then push up
            currentYState = gamepad1.y;
            if (currentYState && !previousYState) {
                // Start pulse only if not already pulsing
                if (!sorterSubsystem.getIsPusherUp()) {
                    sorterSubsystem.setIsPusherUp(true);
                    hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
                    pusherTimer.reset();
                }
            }
            previousYState = currentYState;

            // If the pusher is up AND 500ms have passed since last push, push back down
            if (sorterSubsystem.getIsPusherUp() && pusherTimer.milliseconds() >= 500) {
                hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
                sorterSubsystem.setIsPusherUp(false);
            }

            //if X is pressed, switch outtake motor state
            currentXState = gamepad1.x;
            if (currentXState && !previousXState) {
                isOuttakeMotorOn = !isOuttakeMotorOn;
            }
            previousXState = currentXState;


            //Set hood position and outtake speed with gamepad 2's x, y, and b buttons
            // CLOSE
            if (gamepad2.x) {
                hood.setPosition(CLOSE_HOOD);
                shootSpeed = CLOSE_SHOOT_SPEED;

            }
            //MID
            if (gamepad2.y) {
                hood.setPosition(MID_HOOD);
                shootSpeed = MID_SHOOT_SPEED;
            }

            //FAR
            if (gamepad2.b) {
                hood.setPosition(FAR_HOOD);
                shootSpeed = FAR_SHOOT_SPEED;
            }

            //if start button pressed, reset odometry
            if (gamepad1.start) {
                mecanumCommand.resetPinPointOdometry();
            }

            //if outtake motor is on, then set the max rpm to whatever the shoot speed is
            //if shooter is at max speed, turn light on
            if (isOuttakeMotorOn) {
                shooterSubsystem.setMaxRPM((int) Math.round(shootSpeed));
                if (shooterSubsystem.spinup()) {
                    light.setPosition(0.333);
                } else {
                    light.setPosition(0.0);
                }

            } else {
                shooterSubsystem.stopShooter();
                light.setPosition(0.0);
            }

            //TURRET

            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Name", status.getName());
            telemetry.addData("CPU", "%.1f %%", status.getCpu());
            telemetry.addData("FPS", "%d", (int) status.getFps());
            telemetry.addData("Pipeline", "%d (%s)",
                    status.getPipelineIndex(),
                    status.getPipelineType()
            );

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                if (Math.abs(tx) > 3) {
                    double power = 0.03 * tx;
                    power = Math.max(-1.0, Math.min(1.0, power));
                    llmotor.setPower(-power);
                } else {
                    llmotor.setPower(0);
                }
                telemetry.addData("tx", tx);
                telemetry.update();

            } else {
                if (gamepad2.right_trigger > 0) {
                    llmotor.setPower(-0.5);
                } else if (gamepad2.left_trigger > 0) {
                    llmotor.setPower(0.5);
                } else {
                    llmotor.setPower(0);
                }
            }

                }

            //Add telemetry data for intake motor state, ball color, outtake motor state,
            //Hood position, x, y, and odometry heading, outtake speed, and sorter position
            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.addData("colour?: ", sorterSubsystem.getIsBall());
            telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
            telemetry.addData("Hood pos: ", hoodPos);
            telemetry.addLine("---------------------------------");
            telemetry.addData("X", mecanumCommand.getX());
            telemetry.addData("Y", mecanumCommand.getY());
            telemetry.addData("Theta", mecanumCommand.getOdoHeading());
            telemetry.addData("Outtake speed: ", shootSpeed);
            telemetry.addData("Sorter: ", sorterSubsystem.getCurSorterPositionIndex());
            telemetry.update();

        }
    }


