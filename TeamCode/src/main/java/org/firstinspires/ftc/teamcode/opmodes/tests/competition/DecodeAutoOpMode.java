package org.firstinspires.ftc.teamcode.opmodes.tests.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sorter.SorterSubsystem;

@TeleOp
public class DecodeAutoOpMode extends LinearOpMode {
    Hardware hw;
    private SorterSubsystem sorterSubsystem;

    public void runOpMode() {

        hw = Hardware.getInstance(hardwareMap);
        sorterSubsystem = new SorterSubsystem(hw, this, telemetry);

        hw.colour.enableLed(true);

        String pattern = "GPP";

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Pattern", pattern);
            telemetry.addData("Red", sorterSubsystem.getRed());
            telemetry.addData("Green", sorterSubsystem.getGreen());
            telemetry.addData("Blue", sorterSubsystem.getBlue());
            telemetry.addData("Alpha", sorterSubsystem.getAlpha());
            telemetry.addData("Amount of Artifacts", sorterSubsystem.getNumBalls());
            telemetry.addData("position", sorterSubsystem.getPosition());
            for (int i = 0; i < sorterSubsystem.getNumBalls(); i++) {
                telemetry.addData("Artifacts", sorterSubsystem.getSorterIndex(i));
            }
            telemetry.update();
        }
    }
}
