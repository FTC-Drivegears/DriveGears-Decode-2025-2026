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

        boolean previousAState = false;
        boolean isIntakeMotorOn = false;
        boolean previousXState = false;
        boolean isOuttakeMotorOn = false;
        boolean currentAState = gamepad1.a;
        boolean currentXState = gamepad1.x;

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
                sorterSubsystem.detectColour();
            }

            if (currentXState && !previousXState){
                isOuttakeMotorOn = !isOuttakeMotorOn;

                if (isOuttakeMotorOn){
                    outtakeMotor.setPower(0.8);
                }else{
                    outtakeMotor.setPower(0);
                }
            }

            telemetry.addData("Is intake motor ON?: ", isIntakeMotorOn);
            telemetry.addData("Is outtake motor ON?: ", isOuttakeMotorOn);
            telemetry.update();
        }
    }
}
