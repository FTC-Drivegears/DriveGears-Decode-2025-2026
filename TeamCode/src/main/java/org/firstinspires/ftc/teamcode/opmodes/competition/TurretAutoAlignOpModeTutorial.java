package org.firstinspires.ftc.teamcode.opmodes.competition;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismTutorial;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;


@TeleOp(name = "AutoAlignTest", group = "TeleOp")
public class TurretAutoAlignOpModeTutorial extends OpMode {
    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private TurretMechanismTutorial turret = new TurretMechanismTutorial();

    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private double theta;

    // ----------------- used to auto update P and D ----------------------
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    //index to select the current step size from the array
    int stepIndex = 2;

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        aprilTagWebcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap);

        telemetry.addLine("Initialized all mechanisms");
    }

    public void start(){
        turret.resetTimer();
    }

    @Override
    public void loop(){
        mecanumCommand.processOdometry();

        theta = mecanumCommand.fieldOrientedMove(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        //vision logic
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        turret.update(id20);

        //update P and D on the fly
        // 'B' button cycles through the different step sizes for tuning precision.
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex +1 ) % stepSizes.length;
        }

        //D-pad left/right adjusts the P gain.
        if(gamepad1.dpadLeftWasPressed()){
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        if(gamepad1.dpadRightWasPressed()){
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        //D-pad up/down adjusts the D gain.
        if (gamepad1.dpadUpWasPressed()){
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }

        if(id20 != null){
            telemetry.addData("cur ID", aprilTagWebcam);
        } else{
            telemetry.addLine("No Tag Detected. Stopping Turret Motor");
        }
        telemetry.addLine("----------------");
        telemetry.addData("Tuning P", "%5fv(D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%5fv(D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);
    }
}
