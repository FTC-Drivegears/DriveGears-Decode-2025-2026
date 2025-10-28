package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingCommand;
import org.firstinspires.ftc.teamcode.subsystems.sorting.SortingConstants;


@Autonomous (name = "Movement Test")
public class AutoOpMovementTest extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private SortingCommand sortingCommand;
    private ShooterCommand shooterCommand;

    private double theta;
    private double x;
    private double y;



    private double pos;

    private int stage1 = 0;

    enum SORTING_STATE {
        PPG_1,
        PGP_2,
        GPP_3

    }

    public void unload() throws InterruptedException {

        shooterCommand.shoot(3);
        shooterCommand.stopShoot(3);
//       shooterCommand.shoot(3);
//       shooterCommand.stopShoot(3);
//       shooterCommand.shoot(3);
//       shooterCommand.stopShoot(3);\

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // create Hardware using hardwareMap
        Hardware hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        sortingCommand = new SortingCommand(hw);
        shooterCommand = new ShooterCommand(hw);

        SORTING_STATE sortingState = SORTING_STATE.PPG_1;

        waitForStart();
        //unload();
        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();
            mecanumCommand.moveToPos(30, 20, 30);

            x = mecanumCommand.getOdoX();
            y = mecanumCommand.getOdoY();
            theta = mecanumCommand.getOdoHeading();
            //processPinPoint();


//            switch(sortingState){
//                case PPG_1:
//                    sortingCommand.changePosition(SortingConstants.rotation);
//                    unload();
//                    break;
//
//                case PGP_2:
//                    sortingCommand.changePosition(-SortingConstants.rotation);
//                    unload();
//                    break;
//
//                case GPP_3:
//                    sortingCommand.changePosition(-SortingConstants.rotation);
//                    unload();
//                    break;
//
//            }



//            switch (autoState) {
//                case FIRST_BUCKET:
//                    mecanumCommand.moveToPos(30, 0, 0);
//                    if (mecanumCommand.positionNotReachedYet()) {
//                        autoState = AUTO_STATE.SUB_PICKUP;
//                    }
//                    break;
//                case SUB_PICKUP:
//                    if (mecanumCommand.moveToPos(30, -20, 0)) {
//                        autoState = AUTO_STATE.FINISH;
//                    }
//                    break;
//                case FINISH:
//                    stopRobot();
//                    break;
//            }
        }

    }

    public void processTelemetry(){
        //add telemetry messages here

        telemetry.addLine("---------------------------------");
        telemetry.addData("theta", theta);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();
    }

    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
