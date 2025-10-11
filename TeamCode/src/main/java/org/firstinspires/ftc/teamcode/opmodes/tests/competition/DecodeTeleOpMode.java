package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@TeleOp(name = "DecodeTeleOpMode", group = "TeleOp")
public class DecodeTeleOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;

    private static Hardware instance;

    private Hardware hw;
    private double theta;

    //private DcMotor motorTurret;
//    private DcMotor motorHood;

    private Servo servoSorter;
    private Servo servoPusher;

    private ElapsedTime resetTimer;
    private DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        this.servoSorter = hardwareMap.get(Servo.class, "sorter");
        this.servoPusher = hardwareMap.get(Servo.class, "pusher");

        boolean isIntakeMotorOn = false;
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        resetTimer = new ElapsedTime();

        double setServoSorterPosition = 0;
        double supposedServoSorterPosition = 0;

        servoSorter.setPosition(setServoSorterPosition);

        while (opModeInInit()){
            telemetry.update();
        }

        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                servoPusher.setPosition(1);
                // Launch for as long as the button is pressed
                //motorTurret.setPower(gamepad1.right_trigger);
            }
//            if (gamepad1.left_bumper) {
//                motorHood.setPower(-gamepad1.left_trigger);
//            }
//            else {
//                motorHood.setPower(gamepad1.left_trigger);
//            }

            // Needs correction
            servoPusher.setPosition(0);
            //Thread.sleep(1000);
            if (gamepad1.x) {
                // Go to next sort
                if (servoSorter.getPosition() == 0.3+setServoSorterPosition) {
                    supposedServoSorterPosition = 0.6;
                } else if (servoSorter.getPosition() == 0.6+setServoSorterPosition) {
                    supposedServoSorterPosition = 1;
                } else {
                    supposedServoSorterPosition = 0.3;
                }
            }

            if (gamepad1.y) {
                // Manual override
                setServoSorterPosition += 0.05;
            }

            setServoSorterPosition = (setServoSorterPosition+supposedServoSorterPosition)%1;
            servoSorter.setPosition(setServoSorterPosition);
            // No longer needs correction

            // Intake
            if (gamepad1.right_bumper) {
                //motorTurret.setPower(0.4);
            }

            theta = mecanumCommand.fieldOrientedMove(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );


            if (gamepad1.a) {
                isIntakeMotorOn = !isIntakeMotorOn;
                telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
                telemetry.update();
                if (isIntakeMotorOn){
                    intakeMotor.setPower(-0.35);
                }else{
                    intakeMotor.setPower(0);
                }
            }
            //motorTurret.setPower(0);
        }
    }
    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");


        telemetry.update();
    }
}
