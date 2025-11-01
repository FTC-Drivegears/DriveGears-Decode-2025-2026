package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;

@TeleOp
public class AprilTagRobot extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private Hardware hw;
    double robotangle = 0;
    double tagangle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

            waitForStart();
            while (opModeIsActive()) {
                mecanumCommand.motorProcess();
                mecanumCommand.processOdometry();

                mecanumCommand.moveToPos(0, 0, 30);

            }
        }
    }