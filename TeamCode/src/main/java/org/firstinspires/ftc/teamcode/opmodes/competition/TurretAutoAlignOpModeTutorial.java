package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretMechanismTutorial;

@TeleOp(name = "AutoAlignTest", group = "TeleOp")
public class TurretAutoAlignOpModeTutorial extends OpMode {

    // ---------------- LIMELIGHT ----------------
    private Limelight3A limelight;
    private LLResult llResult;

    // ---------------- SUBSYSTEMS ---------------
    private TurretMechanismTutorial turret = new TurretMechanismTutorial();
    private Hardware hw;
    private MecanumCommand mecanumCommand;

    private double theta;

    // ----------- PD Tuning Step Sizes ----------
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void init() {

        hw = Hardware.getInstance(hardwareMap);
        mecanumCommand = new MecanumCommand(hw);

        turret.init(hardwareMap);

        // Initialize Limelight (name must match config!)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);   // Set your pipeline number here
        limelight.start();

        telemetry.addLine("Initialized with Limelight");
    }

    @Override
    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {

        mecanumCommand.processOdometry();

        theta = mecanumCommand.fieldOrientedMove(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // ---------------- LIMELIGHT VISION ----------------
        llResult = limelight.getLatestResult();

        Double tx = null;

        if (llResult != null && llResult.isValid()) {
            tx = llResult.getTx();   // Horizontal offset from crosshair
        }

        turret.update(tx);

        // ---------------- PD TUNING ----------------

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex +1 ) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        if(gamepad1.dpadRightWasPressed()){
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()){
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }

        // ---------------- TELEMETRY ----------------
        if(tx != null){
            telemetry.addData("Target Visible", true);
            telemetry.addData("tx", tx);
        } else {
            telemetry.addLine("No Target Detected - Turret Stopped");
        }

        telemetry.addLine("----------------");
        telemetry.addData("Tuning P", "%5fv(D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%5fv(D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
