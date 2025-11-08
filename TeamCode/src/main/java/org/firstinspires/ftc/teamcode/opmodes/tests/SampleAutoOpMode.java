//package org.firstinspires.ftc.teamcode.opmodes.tests;
//import android.util.Size;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//
//@Autonomous (name = "Sample Auto")
//public class SampleAutoOpMode extends LinearOpMode {
//    private MecanumCommand mecanumCommand;
//
//    private ElapsedTime resetTimer;
//
//    enum AUTO_STATE {
//        START_POSITION,
//        FIRST_SHOT,
//        COLLECTION,
//        SECOND_SHOT,
//        FINISH
//
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // create Hardware using hardwareMap
//        Hardware hw = Hardware.getInstance(hardwareMap);
//        mecanumCommand = new MecanumCommand(hw);
//        resetTimer = new ElapsedTime();
//
//        AUTO_STATE autoState = AUTO_STATE.START_POSITION;
//        waitForStart();
//        while (opModeIsActive()) {
//            mecanumCommand.motorProcess();
//            mecanumCommand.processOdometry();
//
//            processTelemetry();
//
//            switch (autoState) {
//                case START_POSITION:
//                    mecanumCommand.moveToPos(30, 30, Math.PI/8);
//                    if (mecanumCommand.positionNotReachedYet()) {
//                        autoState = AUTO_STATE.FIRST_SHOT;
//                    }
//                    break;
//                case FIRST_SHOT:
////                    if (mecanumCommand.moveToPos(30, -20, 0)) {
////                        autoState = AUTO_STATE.FINISH;
////                    }
////                    break;
//                case FINISH:
//                    stopRobot();
//                    break;
//            }
//        }
//
//    }
//    public void processTelemetry(){
//        //add telemetry messages here
//        telemetry.addData("resetTimer: ",  resetTimer.milliseconds());
//        telemetry.addLine("---------------------------------");
//        telemetry.addData("X", mecanumCommand.getX());
//        telemetry.addData("Y", mecanumCommand.getY());
//        telemetry.addData("Theta", mecanumCommand.getOdoHeading());
//        telemetry.update();
//    }
//    private void stopRobot() {
//        mecanumCommand.moveGlobalPartialPinPoint(0, 0, 0);
//    }
//}
//
//
