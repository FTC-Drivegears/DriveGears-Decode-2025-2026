package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;


@TeleOp(name = "testingTeleOpMode", group = "TeleOp")
public class testingTeleOpMode extends LinearOpMode {

    // opmodes should only own commands
    private MecanumCommand mecanumCommand;
    private ElapsedTime timer;

    private ElapsedTime resetTimer;

    private Hardware hw;
    private double theta;
    private double x;
    private double y;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        resetTimer = new ElapsedTime();



        // Wait for start button to be pressed
        waitForStart();

        // Loop while OpMode is running
        while (opModeIsActive()) {

            mecanumCommand.processOdometry();

            x = mecanumCommand.getOdoX();
            y = mecanumCommand.getOdoY();

            processTelemetry();

            if (gamepad1.start){
                mecanumCommand.resetPinPointOdometry();
            }


        }

    }
    public void processTelemetry(){
        //add telemetry messages here
        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
        telemetry.addLine("---------------------------------");
        telemetry.addData("theta", theta);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();
    }
}
