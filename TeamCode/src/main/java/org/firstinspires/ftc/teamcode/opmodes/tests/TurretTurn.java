package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class TurretTurn extends LinearOpMode {
    private MecanumCommand mecanumCommand;
    private Hardware hw;
    double target = 0;
    double current;
    double turn; // angle difference
    double pos;


    @Override
    public void runOpMode() throws InterruptedException {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);
        waitForStart();

        while (opModeIsActive()) {
            mecanumCommand.motorProcess();
            mecanumCommand.processOdometry();
            current = mecanumCommand.getOdoHeading();

            pos = current % (2 * Math.PI);
//            turn = ;

            if (gamepad1.start) {
                if (gamepad1.b)
                    if (turn < 180) {
                        mecanumCommand.moveToPos(0, 0, current - target);
                    } else {
                        mecanumCommand.moveToPos(0, 0, current + 2 * Math.PI - turn);
                    }
                mecanumCommand.resetPinPointOdometry();
            }

            if (Math.abs(turn) < Math.PI) {
                turn = current - target;
            } else {

            }


            processTelemetry();

        }
    }

    public void processTelemetry() {
        telemetry.addData("x: ", mecanumCommand.getOdoX());
        telemetry.addData("y: ", mecanumCommand.getOdoY());
        telemetry.addData("current angle: ", current);
        telemetry.addData("turn angle: ", turn);
        telemetry.update();
    }
}