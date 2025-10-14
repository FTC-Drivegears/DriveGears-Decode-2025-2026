package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;



@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private DcMotorEx motorIntake;
    enum AUTO_STATE {
        FIRST_POSITION,
        BALL_PICKUP,
        FINISH

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        motorIntake = hardwareMap.get(DcMotorEx.class, "externalIntake");
        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        AUTO_STATE autoState = AUTO_STATE.FIRST_POSITION;
        waitForStart();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();

            //processPinPoint();

            switch (autoState) {
                case FIRST_POSITION:
                    mecanumCommand.moveToPos(30, 0, 0);
                    motorIntake.setPower(0);  // Turn on intake while moving

                    if (mecanumCommand.positionNotReachedYet()) {
                        autoState = AUTO_STATE.BALL_PICKUP;
                        motorIntake.setPower(0);
                    }
                    break;
                case BALL_PICKUP:
                    if (mecanumCommand.moveToPos(30, 0, 0)) {
                        autoState = AUTO_STATE.FINISH;
                        motorIntake.setPower(0.5);  // Reverse intake to eject
                        sleep(3000);
                        motorIntake.setPower(0);     // Then stop
                    }
                    break;
                case FINISH:
                    motorIntake.setPower(0);  // Ensure intake is stopped
                    stopRobot();
                    break;
            }
        }
    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}