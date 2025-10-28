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
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;

    private Servo pusher;



    @Override
    public void runOpMode() throws InterruptedException {

        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        pusher = hardwareMap.get(Servo.class,"pusher");
        pusher.setPosition(1);

        boolean previousAState = false;
        boolean previousBState = false;
        boolean previousXState = false;
        boolean previousYState = false;
        boolean currentAState;
        boolean currentBState;
        boolean currentXState;
        boolean currentYState;

        boolean isIntakeMotorOn = false;

        boolean isOuttakeMotorOn = false;
        boolean togglePusher = false;
        boolean toggleSorter = false;

        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        sorterSubsystem = new SorterSubsystem(hw, this,telemetry, "PPG");

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
                    intakeMotor.setPower(0.8);
                }else {
                    intakeMotor.setPower(0);
                }
            }
            previousAState = currentAState;

            if (gamepad1.b){
                telemetry.addLine("I see color");
                sorterSubsystem.detectColour();
                telemetry.update();
            }

//            currentBState = gamepad1.b;
//            if (currentBState && !previousBState) {
//                toggleSorter = !toggleSorter;
//
//                if (toggleSorter) {
//                    telemetry.addLine("I SEE COLOR!!!");
//                    sorterSubsystem.detectColour();
//                    telemetry.update();
//                }else{
//                    telemetry.addLine("BUTTON IS OFF!!!");
//                    telemetry.update();
//                }
//            }
//            previousBState = currentBState;

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
                    outtakeMotor.setPower(1);
                }else{
                    outtakeMotor.setPower(0);
                }
            }
            previousXState = currentXState;

            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
            telemetry.update();
        }
    }
}
