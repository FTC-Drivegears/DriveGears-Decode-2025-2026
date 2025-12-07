package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;

@TeleOp(name = "colourSensor")

public class DecodeAutoOpMode extends LinearOpMode {
    Hardware hw;

    private SorterSubsystem sorterSubsystem;

    @Override
    public void runOpMode() {
        hw = Hardware.getInstance(hardwareMap);
        sorterSubsystem = new SorterSubsystem(hw, this, telemetry);

        hw.colour1.enableLed(true);
        hw.colour2.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Red", sorterSubsystem.getRed());
            telemetry.addData("Green", sorterSubsystem.getGreen());
            telemetry.addData("Blue", sorterSubsystem.getBlue());
            telemetry.addData("Alpha", sorterSubsystem.getAlpha());
            telemetry.addData("Amount of Artifacts", sorterSubsystem.getNumBalls());
            telemetry.addData("Red", sorterSubsystem.getRed2());
            telemetry.addData("Green", sorterSubsystem.getGreen2());
            telemetry.addData("Blue", sorterSubsystem.getBlue2());
            telemetry.addData("Alpha", sorterSubsystem.getAlpha2());

            //telemetry.addData("position", sorterSubsystem.getPosition());
           /* for (int i = 0; i < sorterSubsystem.getNumBalls(); i++) {
                telemetry.addData("Artifacts", sorterSubsystem.getSorterIndex(i));
            }*/
            telemetry.update();
            sorterSubsystem.detectColour();

            // After intake it needs to detect 3 balls that enter the robot

            /*
            // After the colour is detected turn to the colour to prepare it for launch
            turnToColour(pattern.get(0), sorterServo);
            sleep(2000);
            // add launch code

            turnToColour(pattern.get(1), sorterServo);
            sleep(2000);
            // add launch code

            turnToColour(pattern.get(2), sorterServo);
            sleep(2000);
            // add launch code
             */
        }
    }
}