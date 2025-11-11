package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;



@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private int stage1 = 0;

    private ElapsedTime resetTimer;



    //Pusher variables
    private static final double PUSHER_UP = 0.75;
    private static final double PUSHER_DOWN = 1.0;
    private static final long PUSHER_TIME = 500;
    private static boolean isPusherUp = false;
    private static final ElapsedTime pusherTimer = new ElapsedTime();

    //Sorter variables
    private static final ElapsedTime sorterTimer = new ElapsedTime();
    private static final double SORTER_FIRST_POS = 0.0;
    private static double initialPos = SORTER_FIRST_POS;

    private static double hoodPos = 1.0;



    @Override
    public void runOpMode() throws InterruptedException {


        }

    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("X", mecanumCommand.getX());
        telemetry.addData("Y", mecanumCommand.getY());
        telemetry.addData("Theta", mecanumCommand.getOdoHeading());
        telemetry.addData("Position reached: ", mecanumCommand.isPositionReached());
        telemetry.update();
    }
    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
    }
}
