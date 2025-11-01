package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class DecodeTeleOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;

    private SorterSubsystem sorterSubsystem;

    private Hardware hw;
    private double theta;
    private DcMotor intake;
    private DcMotor outtake;

    private Servo pusher;



    @Override
    public void runOpMode() throws InterruptedException {

        this.intake = hardwareMap.get(DcMotor.class, "intake");
        this.outtake = hardwareMap.get(DcMotor.class, "outtake");
        pusher = hardwareMap.get(Servo.class,"pusher");
        pusher.setPosition(1);

        boolean previousAState = false;
        boolean previousDpadUpState = false;
        boolean previousDpadDownState = false;
        boolean previousXState = false;
        boolean previousYState = false;
        boolean currentAState;
        boolean currentDpadUpState;
        boolean currentDpadDownState;
        boolean currentXState;
        boolean currentYState;

        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        boolean togglePusher = false;
        boolean mockPurple = false;
        boolean mockGreen = false;

        boolean thisIsPurple = false;
        boolean thisIsGreen = false;

        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        while (opModeInInit()){
            telemetry.update();
        }
        // Wait for start button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            theta = mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            currentAState = gamepad1.a;
            if (currentAState && !previousAState){
                isIntakeMotorOn = !isIntakeMotorOn;

                if (isIntakeMotorOn){
                    intake.setPower(0.8);
                }else {
                    intake.setPower(0);
                }
            }
            previousAState = currentAState;

            // Once camera can understand output patter, revise this initalization.
            sorterSubsystem = new SorterSubsystem(hw,this, telemetry, "pgg");
            String mockInputBalls = "gpg"; //pretend I inputted these balls
            for (char c: mockInputBalls.toCharArray()) {
                sorterSubsystem.intakeBall(c);
            }

            // Outtake ball
            if (gamepad1.dpad_left) {
                sorterSubsystem.outtakeBall();
            }

            if (gamepad1.dpad_right){
                sorterSubsystem.quickFire();
            }

            currentYState = gamepad1.y;
            if (currentYState && !previousYState){
                togglePusher = !togglePusher;

                if (togglePusher){
                    pusher.setPosition(0.4);
                }else{
                    pusher.setPosition(1);
                }
            }
            previousYState = currentYState;

            currentXState = gamepad1.x;
            if (currentXState && !previousXState){
                isOuttakeMotorOn = !isOuttakeMotorOn;

                if (isOuttakeMotorOn){
                    outtake.setPower(1);
                }else{
                    outtake.setPower(0);
                }
            }
            previousXState = currentXState;

            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
            telemetry.update();
        }
    }
}