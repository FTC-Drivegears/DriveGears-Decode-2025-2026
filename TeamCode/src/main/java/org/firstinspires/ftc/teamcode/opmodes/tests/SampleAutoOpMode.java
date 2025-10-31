package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem;


@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private int stage1 = 0;

    enum AUTO_STATE {
        FIRST_BUCKET,
        SUB_PICKUP,
        FINISH

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hw);

        AUTO_STATE autoState = AUTO_STATE.FIRST_BUCKET;
        waitForStart();
        while (opModeIsActive()) {
            mecanumSubsystem.motorProcess();
            mecanumSubsystem.processOdometry();
            //processPinPoint();

            switch (autoState) {
                case FIRST_BUCKET:
                    mecanumSubsystem.moveToPos(30, 0, 0);
                    if (mecanumSubsystem.positionNotReachedYet()) {
                        autoState = AUTO_STATE.SUB_PICKUP;
                    }
                    break;
                case SUB_PICKUP:
                    if (mecanumSubsystem.moveToPos(30, -20, 0)) {
                        autoState = AUTO_STATE.FINISH;
                    }
                    break;
                case FINISH:
                    break;
            }
        }

    }
}
