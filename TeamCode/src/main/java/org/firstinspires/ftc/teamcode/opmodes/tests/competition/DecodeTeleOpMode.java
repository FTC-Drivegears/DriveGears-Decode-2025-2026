package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    @Override
    public void runOpMode() throws InterruptedException {

        this.intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");

        boolean isIntakeMotorOn = false;
        boolean isOuttakeMotorOn = false;
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        sorterSubsystem = new SorterSubsystem(hw, this,telemetry);

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

            if(gamepad1.a && isIntakeMotorOn){
                isIntakeMotorOn = false;
            } else if (gamepad1.a){
                isIntakeMotorOn = true;
            }

            if (gamepad1.b){
                sorterSubsystem.detectColour();
            }

            if (gamepad1.x && isOuttakeMotorOn){
                isOuttakeMotorOn = false;
            } else if (gamepad1.x){
                isOuttakeMotorOn = true;
            }

            //Press the button once to activate
            if(isOuttakeMotorOn){
                outtakeMotor.setPower(1);
            } else{
                outtakeMotor.setPower(0);
            }

            if (isIntakeMotorOn){
                intakeMotor.setPower(0.8);
            } else{
                intakeMotor.setPower(0);
            }

            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.update();
//            isIntakeMotorOn = gamepad1.a;
//            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
//            telemetry.update();
//            if (isIntakeMotorOn){
//                intakeMotor.setPower(-0.5);
//            } else{
//                intakeMotor.setPower(0);
//            }
        }
    }
}
